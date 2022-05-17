import json
import math
from labutils.common import *
from typing import Any, List, NamedTuple, Optional, Union, Tuple
from dataclasses import dataclass, field
from collections import defaultdict
import shutil


class BaseCommand:
    __subclasses__: list = []
    __instances__: list = []
    __handlers__: list = []

    id: str = "unique_command_id"
    name: str = "command name"
    info: str = "long command description"
    index: int = 0  # Command position in dropdown
    handlers: list = []

    def __init__(self):
        self.handlers = []

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        log(f"Registered command subclass: {cls}", "debug")
        BaseCommand.__subclasses__.append(cls)

    @classmethod
    def register_all(cls):
        BaseCommand.__subclasses__.sort(key=lambda x: x.index)
        for cmd_class in BaseCommand.__subclasses__:
            instance = cmd_class()
            log(f"Registering command (id: {instance.id})", "debug")
            cmd = create_command(instance.id, instance.name, instance.info)
            add_handler(cmd.commandCreated, instance.setup, BaseCommand.__handlers__)
            BaseCommand.__instances__.append(instance)

    def setup(self, args: adsk.core.CommandCreatedEventArgs):
        log(f"Setup command (id: {self.id})", "debug")
        add_handler(args.command.execute, self.execute, self.handlers)
        add_handler(args.command.destroy, self.destroy, self.handlers)
        add_handler(args.command.inputChanged, self.update, self.handlers)
    
    def update(self, args: adsk.core.InputChangedEvent):
        log(f"Command inputs updated (id: {self.id})", "debug")

    def execute(self, args: adsk.core.CommandEventArgs):
        log(f"Executing command (id: {self.id})", "debug")

    def destroy(self, args: adsk.core.CommandEventArgs):
        log(f"Destroying command (id: {self.id})", "debug")
        self.handlers.clear()


class OTSMassCommand(BaseCommand):
    id = f"{PLUGIN_NAME}_OTSMassCommand"
    name = "Set off-the-shelf mass"
    info = "Updates physical materials to match off-the-shelf mass of entire component"
    index = 1

    def setup(self, args: adsk.core.CommandCreatedEventArgs):
        super().setup(args)
        inputs = args.command.commandInputs
        body_select = inputs.addSelectionInput("body_select", "Bodies", "")
        body_select.addSelectionFilter("SolidBodies")
        body_select.setSelectionLimits(1)
        default = adsk.core.ValueInput.createByString("500 g")
        inputs.addValueInput("mass_input", "Mass", "kg", default)

    def execute(self, args: adsk.core.CommandEventArgs):
        super().execute(args)
        design = app.activeProduct
        inputs = args.command.commandInputs
        body_select = inputs.itemById("body_select")
        mass_input = inputs.itemById("mass_input")  # kg
        selected_bodies = [
            body_select.selection(i).entity
            for i in range(body_select.selectionCount)
        ]

        materials = {}
        material_to_bodies = defaultdict(list)
        material_total_volume = defaultdict(int)
        total_volume = 0
        for body in selected_bodies:
            props = body.getPhysicalProperties()
            mid = body.material.id
            materials[mid] = body.material
            material_to_bodies[mid].append(body)
            material_total_volume[mid] += props.volume
            total_volume += props.volume

        global_density =  mass_input.value / total_volume * 1e6  # kg/m3
        log(f"Global density: {global_density} kg/cm3")
        for mid, material in materials.items():
            bodies = material_to_bodies[mid]
            if not material.name.endswith(" (OTS Mass)"):
                new_name = material.name + " (OTS Mass)"
                source_material = material
                material = design.materials.addByCopy(material, new_name)
                for body in bodies:
                    body.material = material
                try:
                    source_material.deleteMe()
                except Exception:
                    log("Failed to delete source material")
            density_prop = material.materialProperties.itemById("structural_Density")
            density_prop.value = global_density  # kg/m3


class ShowTransformCommand(BaseCommand):
    id = f"{PLUGIN_NAME}_ShowTransformCommand"
    name = "Show occurrence transform"
    index = 2

    def setup(self, args: adsk.core.CommandCreatedEventArgs):
        super().setup(args)
        inputs = args.command.commandInputs
        self.comp_select = inputs.addSelectionInput("comp_select", "Component", "")
        self.comp_select.addSelectionFilter("Occurrences")
        self.comp_select.setSelectionLimits(1, 1)
        self.text_label_1 = inputs.addTextBoxCommandInput("text_label_1", "transform", "", 1, True)
        self.text_label_2 = inputs.addTextBoxCommandInput("text_label_2", "transform2", "", 1, True)

    def update(self, args: adsk.core.InputChangedEvent):
        super().update(args)
        if self.comp_select.selectionCount != 1:
            return
        occ = self.comp_select.selection(0).entity
        self.text_label_1.text = f"{self._get_offset(occ.transform)}"
        self.text_label_2.text = f"{self._get_offset(occ.transform2)}"
        
    def _get_offset(self, transform):
        trans = transform.asArray()
        origin = [0, 6.5, 0]
        offset = self._trans(trans, origin)
        return [round(x * 10, 1) for x in offset]

    def _trans(self, M, a):
        ex = [M[0], M[4], M[8]]
        ey = [M[1], M[5], M[9]]
        ez = [M[2], M[6], M[10]]
        oo = [M[3], M[7], M[11]]
        b = [0, 0, 0]
        for i in range(3):
            b[i] = a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i] + oo[i]
        return b


@dataclass(repr=False)
class JointWrapper:
    obj: Union[adsk.fusion.Joint, adsk.fusion.RigidGroup]
    name: str
    is_suppressed: bool
    parent: "ComponentWrapper"
    components: List["ComponentWrapper"] = field(default_factory=list)
    type: str = None
    axis: List[float] = None
    origin: List[float] = None
    limits: List[float] = None

    # @property
    # def is_fixed(self):
    #     if isinstance(self.obj, adsk.fusion.RigidGroup):
    #         return True
    #     if isinstance(self.obj.jointMotion, adsk.fusion.RigidJointMotion):
    #         return True
    #     return False

    @classmethod
    def construct(cls, obj, parent: "ComponentWrapper"):
        if isinstance(parent.obj, adsk.fusion.Occurrence):
            obj = obj.createForAssemblyContext(parent.obj)
        if obj.timelineObject.isRolledBack:
            # TODO: Better way?
            log(f"Joint {obj.name!r} is rolled back")
            return None
        return cls(
            obj=obj,
            name=obj.name,
            is_suppressed=obj.isSuppressed,
            parent=parent,
        )

    def init_components(self, all_components):
        raw_occurrences = (
            self.obj.occurrences
            if isinstance(self.obj, adsk.fusion.RigidGroup)
            else [self.obj.occurrenceOne, self.obj.occurrenceTwo]
        )
        for occ in raw_occurrences:
            if occ is None:
                continue
            path = occ.fullPathName
            if self.parent.is_external and not path.startswith(self.parent.path):
                path = self.parent.path + "+" + path
            for comp in all_components:
                if comp.path == path:
                    self.components.append(comp)
                    comp.used_in_joints.append(self)
                    break
            else:
                raise RuntimeError(f"Can't find component: {path!r}")


@dataclass(repr=False)
class BodyWrapper:
    obj: adsk.fusion.BRepBody
    name: str
    is_visible: bool
    parent: "ComponentWrapper"

    @classmethod
    def construct(cls, obj, parent):
        if isinstance(parent.obj, adsk.fusion.Occurrence):
            obj = obj.createForAssemblyContext(parent.obj)
        return cls(
            obj=obj,
            name=obj.name,
            is_visible=obj.isVisible,
            parent=parent,
        )


@dataclass(repr=False)
class ComponentWrapper:
    obj: Union[adsk.fusion.Component, adsk.fusion.Occurrence]
    name: str
    path: str
    depth: int
    is_visible: bool
    is_external: bool
    group: Optional[int] = None
    children: List["ComponentWrapper"] = None
    flat_children: List["ComponentWrapper"] = None
    joints: List[JointWrapper] = None
    bodies: List[BodyWrapper] = None
    used_in_joints: List[JointWrapper] = field(default_factory=list)

    @classmethod
    def construct(cls, obj, depth: int = 0):
        if isinstance(obj, adsk.fusion.Component):
            return cls._construct(
                obj=obj,
                comp=obj,
                path="__root__",
                is_visible=True,
                is_external=False,
                raw_children=obj.occurrences,
                depth=depth,
            )
        elif isinstance(obj, adsk.fusion.Occurrence):
            return cls._construct(
                obj=obj,
                comp=obj.component,
                path=obj.fullPathName,
                is_visible=obj.isVisible,
                is_external=obj.isReferencedComponent,
                raw_children=obj.childOccurrences,
                depth=depth,
            )
        else:
            raise RuntimeError("Unsupported object")

    @classmethod
    def _construct(cls, obj, comp, path, is_visible, is_external, raw_children, depth):
        instance = cls(
            obj=obj,
            name=obj.name,
            path=path,
            depth=depth,
            is_visible=is_visible,
            is_external=is_external,
        )
        instance.children = [cls.construct(x, depth=depth + 1) for x in raw_children]
        instance.flat_children = []
        for child in instance.children:
            instance.flat_children.append(child)
            instance.flat_children.extend(child.flat_children)
        raw_bodies = safe_get(comp, "bRepBodies", [])
        instance.bodies = [BodyWrapper.construct(x, instance) for x in raw_bodies]
        raw_joints = [
            *safe_get(comp, "joints", []),
            *safe_get(comp, "asBuiltJoints", []),
            *safe_get(comp, "rigidGroups", []),
        ]
        instance.joints = [JointWrapper.construct(x, instance) for x in raw_joints]
        instance.joints = [x for x in instance.joints if x is not None]
        return instance

    def __eq__(self, other):
        return self.path == other.path

    def __hash__(self):
        return hash(self.path)


@dataclass(repr=False)
class GroupWrapper:
    id: int
    components: List[ComponentWrapper]
    mesh_file: str = None
    mass: float = None
    centroid: List[float] = None
    inertia: List[float] = None


@dataclass()
class CollisionWrapper:
    name: str
    type: str
    center: List[float]
    rotation: List[float]
    radius: float = None
    length: float = None
    size: List[float] = None


class SmartExportCommand(BaseCommand):
    id = f"{PLUGIN_NAME}_SmartExportCommand"
    name = "Smart export for URDF integration"
    info = "Exports meshes, masses, joints and collision primitives that can be used for URDF construction"

    def setup(self, args: adsk.core.CommandCreatedEventArgs):
        super().setup(args)
        self.design = app.activeProduct
        self.root = self.design.rootComponent
        self.inputs = args.command.commandInputs
        self._init_inputs()

        self.input_path = None
        self.output_path = None
        self.all_components = []
        self.all_joints = []
        self.all_bodies = []
        self.all_groups = []
        self.selected_components: List[ComponentWrapper] = []
        self.selected_joints: List[JointWrapper] = []
        self.selected_groups: List[GroupWrapper] = []
        self.selected_collisions: List[CollisionWrapper] = []
    
    def _init_inputs(self):
        self.comp_select = self.inputs.addSelectionInput("comp_select", "Components", "")
        self.comp_select.addSelectionFilter("Occurrences")
        self.comp_select.setSelectionLimits(1)
        self.joint_select = self.inputs.addSelectionInput("joint_select", "Joints", "")
        self.joint_select.addSelectionFilter("Joints")
        self.joint_select.setSelectionLimits(1)
        self.coll_select = self.inputs.addSelectionInput("coll_select", "Collisions", "")
        self.coll_select.addSelectionFilter("Features")
        self.coll_select.setSelectionLimits(1)     
        self.input_file_btn = self.inputs.addBoolValueInput("input_file_btn", "Input OBJ File", False)
        self.input_file_btn.text = "SELECT"
        self.output_dir_btn = self.inputs.addBoolValueInput("output_dir_btn", "Output directory", False)
        self.output_dir_btn.text = "SELECT"
        self.update_obj_btn = self.inputs.addBoolValueInput("update_obj_btn", "Update OBJ files", True)

    def update(self, args: adsk.core.InputChangedEvent):
        super().update(args)

        if args.input.id == "input_file_btn":
            dialog = ui.createFileDialog()
            dialog.title = "Select input OBJ File"
            dialog.filter = "OBJ files (*.obj)"
            result = dialog.showOpen()
            if result == adsk.core.DialogResults.DialogOK:
                path = Path(dialog.filename)
                if path.exists():
                    self.input_path = path
                    self.input_file_btn.text = path.name
                    if self.output_path is None:
                        self.output_path = path.parent
                        self.output_dir_btn.text = self.output_path.name

        if args.input.id == "output_dir_btn":
            dialog = ui.createFolderDialog()
            dialog.title = "Select output directory"
            result = dialog.showDialog()
            if result == adsk.core.DialogResults.DialogOK:
                path = Path(dialog.folder)
                path.mkdir(parents=True, exist_ok=True)
                self.output_path = path
                self.output_dir_btn.text = path.name

    def execute(self, args: adsk.core.CommandEventArgs):
        super().execute(args)
        if self.input_path is None:
            self._fail("No input OBJ file selected")
        if self.output_path is None:
            self._fail("No output directory selected")

        self._build_document_index()
        self._get_selected_items()
        self._find_rigid_groups()
        self._get_selected_groups()
        self._split_obj_file()
        self._get_physical_params()
        self._get_joint_info()
        self._write_json_output()

    def _fail(self, message):
        ui.messageBox(message, PLUGIN_NAME)
        raise RuntimeError(f"Abort: {message}")

    def _build_document_index(self):
        log("Building full document index...")
        root = ComponentWrapper.construct(self.root)
        self.all_components = [root]
        hash_lookup = {root}
        for comp in root.flat_children:
            if comp in hash_lookup:
                raise RuntimeError(f"Component hash collision: {comp.path!r}")
            self.all_components.append(comp)
            hash_lookup.add(comp)
        self.all_joints = sum((x.joints for x in self.all_components), start=[])
        for joint in self.all_joints:
            joint.init_components(self.all_components)
        self.all_bodies = sum((x.bodies for x in self.all_components), start=[])
        log(
            f"Index ready: {len(self.all_components)} components, "
            f"{len(self.all_bodies)} bodies, {len(self.all_joints)} joints"
        )

    def _get_selected_items(self):
        # Components
        for i in range(self.comp_select.selectionCount):
            occ = self.comp_select.selection(i).entity
            for comp in self.all_components:
                if comp.path == occ.fullPathName:
                    self.selected_components.append(comp)
                    break
            else:
                raise RuntimeError(f"Can't find component: {occ.fullPathName!r}")

        # Joints
        for i in range(self.joint_select.selectionCount):
            raw_joint = self.joint_select.selection(i).entity
            context = raw_joint.assemblyContext
            parent_path = "__root__" if context is None else context.fullPathName
            for joint in self.all_joints:
                if parent_path == joint.parent.path and joint.name == raw_joint.name:
                    self.selected_joints.append(joint)
                    break
            else:
                raise RuntimeError(
                    f"Can't find joint by parent + name: "
                    f"{parent_path!r} + {raw_joint.name!r}"
                )

        # Collisions
        log(f"Processing collisions:")
        for i in range(self.coll_select.selectionCount):
            feature = self.coll_select.selection(i).entity
            if isinstance(feature, adsk.fusion.CylinderFeature):
                coll = self._get_cylinder_params(feature)
            elif isinstance(feature, adsk.fusion.BoxFeature):
                coll = self._get_box_params(feature)
            else:
                log(f"Unknown feature type: {type(feature).__name__}")
                continue
            self.selected_collisions.append(coll)
            log(f"- type: {coll.type!r}")
            log(f"  name: {coll.name!r}")
            log(f"  center: {[round(x * 10) for x in coll.center]}")
            log(f"  rotation: {[round(x, 3) for x in coll.rotation]}")
            if coll.radius is not None:
                log(f"  radius: {round(coll.radius * 10)}")
                log(f"  length: {round(coll.length * 10)}")
            if coll.size is not None:
                log(f"  size: {[round(x, 3) for x in coll.size]}")

        log(f"Total selected components: {len(self.selected_components)}")
        log(f"Total selected joints: {len(self.selected_joints)}")

    def _get_cylinder_params(self, feature):
        end_points = []
        axis = None
        radius = None
        center = None
        for face in feature.faces:
            if isinstance(face.geometry, adsk.core.Cylinder):
                axis = face.geometry.axis
                radius = face.geometry.radius
                center = face.centroid
            else:
                end_points.append(face.centroid)
        length = end_points[0].distanceTo(end_points[1])
        # TODO: Fix
        # unit_vectors = [
        #     adsk.core.Vector3D.create(1, 0, 0),
        #     adsk.core.Vector3D.create(0, 1, 0),
        #     adsk.core.Vector3D.create(0, 0, 1),
        # ]
        # rotation = [axis.angleTo(x) for x in unit_vectors]
        rotation = [0, 0, math.pi / 2]
        return CollisionWrapper(
            name=feature.parentComponent.name,
            type="cylinder",
            center=center.asArray(),
            rotation=rotation,
            radius=radius,
            length=length,
        )

    def _get_box_params(self, feature):
        # TODO: Support rotations?
        body = next(iter(feature.bodies))
        bbox = body.boundingBox
        x1, y1, z1 = bbox.minPoint.asArray()
        x2, y2, z2 = bbox.maxPoint.asArray()
        center = [(x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2]
        size = [x2 - x1, y2 - y1, z2 - z1]
        return CollisionWrapper(
            name=feature.parentComponent.name,
            type="box",
            center=center,
            rotation=[0, 0, 0],
            size=size,
        )

    def _find_rigid_groups(self):
        self.all_groups = []
        for i, comp in enumerate(self.all_components):
            comp.group = i
        for joint in self.all_joints:
            if not self._is_fixed_joint(joint):
                continue
            main_group_id = joint.components[0].group
            for comp in joint.components:
                child_group_id = comp.group
                for sibling in self.all_components:
                    if sibling.group == child_group_id:
                        sibling.group = main_group_id

        unique_group_ids = set(x.group for x in self.all_components)
        orphans = {}
        for group_id in unique_group_ids:
            comps = [x for x in self.all_components if x.group == group_id]
            if len(comps) == 1:
                comp = comps[0]
                comp.group = None
                orphans[comp.path] = comp
                continue
            self.all_groups.append(comps)

        for i, group in enumerate(self.all_groups):
            for comp in group:
                comp.group = i
        
        for comp in sorted(orphans.values(), key=lambda x: -x.depth):
            comp.group = None
            if self._check_orphan(comp):
                orphans.pop(comp.path)
                if comp.group is None:
                    comp.group = -1  # Means "no group, but not an orphan"
        for comp in self.all_components:
            if comp.group == -1:
                comp.group = None
        if orphans:
            log(f"Found {len(orphans)} unfixed components:\n", "error")
            for comp in list(orphans.values())[:10]:
                log(f"- {comp.name}\n", "error")
        
        log(f"Total {len(self.all_groups)} rigid groups. Exemplars:")
        for i, group in enumerate(self.all_groups):
            log(f"[{i}] {group[0].name!r}")

    def _is_fixed_joint(self, joint: JointWrapper):
        if joint.is_suppressed:
            return False
        if isinstance(joint.obj, adsk.fusion.RigidGroup):
            return True
        if isinstance(joint.obj.jointMotion, adsk.fusion.RigidJointMotion):
            return True
        return joint not in self.selected_joints

    def _check_orphan(self, comp: ComponentWrapper):
        if comp.path == "__root__":
            return True
        if not comp.is_visible:
            return True
        if comp.used_in_joints:
            return True
        if (
            not comp.bodies
            and comp.children
            and all(x.group is not None for x in comp.children)
        ):
            # Component with no bodies has children and all children are not orphans
            children_groups = set(x.group for x in comp.flat_children)
            if None not in children_groups and len(children_groups) == 1:
                # All children and subchildren are in the same group
                comp.group = comp.children[0].group
                self.all_groups[comp.group].append(comp)
            return True
        # TODO: Add option to automatically fix orphan children to parent
        return False
    
    def _get_selected_groups(self):
        moveable_group_ids = set()
        for joint in self.selected_joints:
            affected_groups = set(x.group for x in joint.components)
            if len(affected_groups) == 1:
                exemplar = self.all_groups[next(iter(affected_groups))][0]
                msg = (
                    f"Selected joint {joint.name!r} conflicts with rigid group. "
                    f"Exemplar group member: {exemplar.name!r}"
                )
                self._fail(msg)
            moveable_group_ids.update(affected_groups)
        if None in moveable_group_ids:
            self._fail("One of selected components is an orphan")
        self.selected_groups = []
        for group_id in moveable_group_ids:
            group = GroupWrapper(id=group_id, components=self.all_groups[group_id])
            self.selected_groups.append(group)
        log(f"Exported groups: {moveable_group_ids}")
    
    def _split_obj_file(self):
        log(f"Exporting meshes:")
        main_mtl_path = self.input_path.with_suffix(".mtl")
        visible_bodies = [x for x in self.all_bodies if x.is_visible]
        for i, group in enumerate(self.selected_groups):
            selected_components = []
            for comp in group.components:
                if comp in self.selected_components:
                    selected_components.append(comp)
            if not selected_components:
                log(f"[{i}] Skip (no selected components)")
                group.mesh_file = None
                continue
            group_output_path = self.output_path / f"group{i}.obj"
            group.mesh_file = group_output_path.name
            if not self.update_obj_btn.value:
                log(f"[{i}] Expected mesh: {group_output_path.name}")
                continue
            body_indices = set()
            for comp in selected_components:
                for body in comp.bodies:
                    if body in visible_bodies:
                        body_indices.add(visible_bodies.index(body))
            group_output_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(main_mtl_path, group_output_path.with_suffix(".mtl"))
            with open(self.input_path, "r") as input_file:
                with open(group_output_path, "w") as output_file:
                    output_file.write(f"mtllib group{i}.mtl\n\n")
                    # https://stackoverflow.com/questions/37785215/
                    self._write_group_model(input_file, output_file, body_indices)
            log(f"[{i}] Mesh created: {group_output_path.name}")

    def _write_group_model(self, input_file, output_file, body_indices):
        # TODO: Probably move it to separate module
        g_counter = -1
        v_counter = 0
        vt_counter = 0
        vn_counter = 0

        faces_block_start = False
        prev_material = None
        material_activated = False
        
        line = input_file.readline()
        while line:
            if line.startswith("g "):
                g_counter += 1
                faces_block_start = False
                material_activated = False
            if line.startswith("usemtl "):
                prev_material = line
                material_activated = True
            if g_counter in body_indices:
                if line.startswith("f "):
                    # Write material before faces block, if skipped
                    if not faces_block_start and not material_activated:
                        output_file.write(prev_material)
                        faces_block_start = True
                    new_line = "f"
                    for token in line.split()[1:]:
                        v, vt, vn = map(int, token.split("/"))
                        v -= v_counter
                        vt -= vt_counter
                        vn -= vn_counter
                        new_line += f" {v}/{vt}/{vn}"
                    new_line += "\n"
                    line = new_line
                output_file.write(line)
            else:
                if line.startswith("v "):
                    v_counter += 1
                if line.startswith("vt "):
                    vt_counter += 1
                if line.startswith("vn "):
                    vn_counter += 1
            line = input_file.readline()

    def _get_physical_params(self):
        log(f"Processing group physical params:")
        for group in self.selected_groups:
            components = set(group.components)
            bodies = []
            for comp in sorted(group.components, key=lambda x: -x.depth):
                if not comp.children:
                    continue
                if all(x.group == group.id for x in comp.children):
                    # All in same group - take only parent component
                    for child in comp.children:
                        components.discard(child)
                else:
                    # Different groups - take children + bodies separately
                    components.discard(comp)
                    bodies.extend(comp.bodies)
            self._merge_physical_params(group, [*components, *bodies])
            log(f"[{group.id}] Group params:")
            log(f"  Original size: {len(group.components)} components")
            log(f"  Merged size: {len(components)} components + {len(bodies)} bodies")
            log(f"  Group mass: {group.mass * 1000:.1f} g")
            log(f"  Group centroid: {[round(x * 10) for x in group.centroid]} mm")
            log(f"  Group inertia: {[round(x * 1e5) for x in group.inertia]} g mm2")

    def _merge_physical_params(self, group: GroupWrapper, objects):
        accuracy = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy  # TODO: ?
        params = [x.obj.getPhysicalProperties(accuracy) for x in objects]
        group.mass = sum(x.mass for x in params)
        gx = gy = gz = 0
        for param in params:
            x, y, z = param.centerOfMass.asArray()
            gx += param.mass * x
            gy += param.mass * y
            gz += param.mass * z
        group.centroid = (gx / group.mass, gy / group.mass, gz / group.mass)
        total_inertia = [0, 0, 0, 0, 0, 0]  # xx, yy, zz, xy, yz, xz
        for param in params:
            _, *inertia_at_origin = param.getXYZMomentsOfInertia()
            for i, x in enumerate(inertia_at_origin):
                total_inertia[i] += x
        # Translate inertia from origin to centroid
        x, y, z = group.centroid
        d2 = [-y * y - z * z, -x * x - z * z, -x * x - y * y, x * y, y * z, x * z]
        group.inertia = [old + group.mass * t for old, t in zip(total_inertia, d2)]

    def _get_joint_info(self):
        log(f"Processing joint info:")
        for joint in self.selected_joints:
            log(f"[*] Joint {joint.name!r}:")
            motion = joint.obj.jointMotion
            if isinstance(motion, adsk.fusion.RevoluteJointMotion):
                joint.type = "continuous"
                limits = motion.rotationLimits
                joint.limits = [0, 0]  # radians
                if limits.isMaximumValueEnabled	and limits.isMinimumValueEnabled:
                    joint.type = "revolute"
                    joint.limits[0] = limits.minimumValue
                    joint.limits[1] = limits.maximumValue
                joint.axis = motion.rotationAxisVector.asArray()
                if isinstance(joint.obj, adsk.fusion.AsBuiltJoint):
                    joint.origin = self._get_asbuilt_joint_origin(joint.obj)

                    results = {
                        'Front Left Wheel Steering': [106.683, -13.294, 202.5],
                        'Front Left Wheel Spin': [114.183, -13.294, 202.5],
                        'Front Right Wheel Steering': [-106.683, -13.294, 202.5],
                        'Front Right Wheel Spin': [-114.183, -13.294, 202.5],
                        'Rear Left Wheel Spin': [111.683, -13.294, -202.5],
                        'Rear Right Wheel Spin': [-111.683, -13.294, -202.5],
                    }

                    if joint.name in results:
                        res = [x * 10 for x in joint.origin]
                        true_res = results[joint.name]
                        assert self._allclose(res, true_res, 0.1)
                else:
                    log("  Mode: normal")
                    joint.origin = self._get_true_joint_origin(joint.obj)
                log(f"  Type: {joint.type}")
                log(f"  Limits: {[round(x, 2) for x in joint.limits]}")
                log(f"  Axis: {[round(x, 3) for x in joint.axis]}")
                log(f"  Origin: {[round(x * 10, 3) for x in joint.origin]}")
            elif isinstance(motion, adsk.fusion.BallJointMotion):
                joint.type = "ball"
                if isinstance(joint.obj, adsk.fusion.AsBuiltJoint):
                    log("  Mode: asbuilt")
                    joint.origin = self._get_asbuilt_joint_origin(joint.obj)
                else:
                    log("  Mode: normal")
                    joint.origin = self._get_true_joint_origin(joint.obj)
                log(f"  Type: {joint.type}")
                log(f"  Origin: {[round(x * 10, 3) for x in joint.origin]}")
            else:
                raise RuntimeError(f"Unsupported joint type: {joint.name}")

    def _write_json_output(self):
        components = []
        for comp in self.all_components:
            item = {
                "id": comp.path,
                "name": comp.name,
                "group": comp.group,
            }
            components.append(item)

        groups = []
        for i, group in enumerate(self.selected_groups):
            item = {
                "id": i,
                "mesh": group.mesh_file,
                "mass": group.mass,  # kg
                "centroid": self._flip(x / 100 for x in group.centroid),  # cm -> m
                "inertia": self._flip(x / 10000 for x in group.inertia),  # kg * cm^2 -> kg * m^2
            }
            groups.append(item)

        joints = []
        for joint in self.selected_joints:
            affected_groups = set(x.group for x in joint.components)
            item = {
                "id": joint.name,
                "groups": list(affected_groups),
                "type": joint.type,
                "origin": self._flip(x / 100 for x in joint.origin),
            }
            if joint.axis is not None:
                item["axis"] = self._flip(joint.axis)
            if joint.limits is not None:
                item["limits"] = joint.limits
            joints.append(item)

        collisions = []
        for coll in self.selected_collisions:
            item = {
                "id": coll.name,
                "type": coll.type,
                "center": self._flip(x / 100 for x in coll.center),
                "rotation": self._flip(coll.rotation),
            }
            if coll.radius is not None:
                item["radius"] = coll.radius / 100
                item["length"] = coll.length / 100
            if coll.size is not None:
                item["size"] = self._flip(x / 100 for x in coll.size)
            collisions.append(item)
                
        with open(self.output_path / "export.json", "w") as file:
            data = {
                "components": components,
                "groups": groups,
                "joints": joints,
                "collisions": collisions,
            }
            json.dump(data, file, ensure_ascii=False, indent=4)
        log(f"Data saved to export.json")

    def _flip(self, *args):
        if len(args) == 1:  # Unpacked array / generator
            args = list(args[0])
        if len(args) == 3:  # Point or vector
            x, y, z = args
            return [z, x, y]
        elif len(args) == 6:  # Inertia
            xx, yy, zz, xy, yz, xz = args
            return [zz, xx, yy, xz, xy, yz]
        else:
            raise RuntimeError(f"Unexpected args len: {len(args)}")

    def _trans(self, M, a):
        ex = [M[0], M[4], M[8]]
        ey = [M[1], M[5], M[9]]
        ez = [M[2], M[6], M[10]]
        oo = [M[3], M[7], M[11]]
        b = [0, 0, 0]
        for i in range(3):
            b[i] = a[0] * ex[i] + a[1] * ey[i] + a[2] * ez[i] + oo[i]
        return b

    def _allclose(self, v1, v2, tol=1e-6):
        return all(abs(a - b) < tol for a, b in zip(v1, v2))

    def _get_asbuilt_joint_origin(self, joint):
        context = joint.assemblyContext
        if context is not None:
            pt = joint.assemblyContext.transform2.asArray()
        else:
            pt = adsk.core.Matrix3D.create().asArray()
        at = joint.occurrenceOne.transform2.asArray()
        bt = joint.occurrenceTwo.transform2.asArray()
        origin = joint.geometry.origin.asArray()
        if self._allclose(at, bt):
            origin = self._trans(pt, origin)
            origin = self._trans(at, origin)
        return origin

    def _get_true_joint_origin(self, joint):
        xyz_from_one_to_joint = joint.geometryOrOriginOne.origin.asArray()
        xyz_from_two_to_joint = joint.geometryOrOriginTwo.origin.asArray()
        xyz_of_one = joint.occurrenceOne.transform.translation.asArray()
        M_two = joint.occurrenceTwo.transform.asArray()

        case1 = self._allclose(xyz_from_two_to_joint, xyz_from_one_to_joint)
        case2 = self._allclose(xyz_from_two_to_joint, xyz_of_one)
        if case1 or case2:
            return xyz_from_two_to_joint
        else:
            return self._trans(M_two, xyz_from_two_to_joint)
