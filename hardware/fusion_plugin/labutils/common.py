import adsk.core
import traceback
from pathlib import Path
import sys

PLUGIN_NAME = "LabUtils"
COMPANY_NAME = "TheLab"
WORKSPACE_ID = "FusionSolidEnvironment"
DEBUG_MODE = False

MAIN_TAB_ID = "ToolsTab"  # aka "UTILITIES"
MAIN_PANEL_ID = f"{PLUGIN_NAME}_MAIN_PANEL_ID"
MAIN_PANEL_NAME = PLUGIN_NAME

LOG_LEVELS = {
    "debug": adsk.core.LogLevels.InfoLogLevel,
    "info": adsk.core.LogLevels.InfoLogLevel,
    "warning": adsk.core.LogLevels.WarningLogLevel,
    "error": adsk.core.LogLevels.ErrorLogLevel,
}


app = adsk.core.Application.get()
ui = app.userInterface
if app is None or ui is None:
    raise RuntimeError("Can't find fusion app")


def log(message, level="info"):
    if level.lower() == "debug" and not DEBUG_MODE:
        return
    message = f"[{PLUGIN_NAME} :: {level.upper()}] {message}"
    level = LOG_LEVELS.get(level.lower(), LOG_LEVELS["info"])
    app.log(message, level, adsk.core.LogTypes.ConsoleLogType)


def catch_errors(func):
    def deco(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            log(traceback.format_exc(), level="error")
    return deco


def delete_command(id):
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(MAIN_PANEL_ID)
    cmd_control = panel.controls.itemById(id)
    if cmd_control:
        cmd_control.deleteMe()
        log(f"Deleted cmd control (id: {id})", "debug")
    cmd_def = ui.commandDefinitions.itemById(id)
    if cmd_def:
        cmd_def.deleteMe()
        log(f"Deleted cmd definition (id: {id})", "debug")


def create_command(id, name, info):
    delete_command(id)
    icon_path = Path(__file__).parent.parent / "resources"
    cmd = ui.commandDefinitions.addButtonDefinition(id, name, info, str(icon_path))
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(MAIN_PANEL_ID)
    panel.controls.addCommand(cmd)
    log(f"Created new command (id: {id})", "debug")
    return cmd


def add_handler(event, callback, handler_list):
    module = sys.modules[event.__module__]
    handler_type = module.__dict__[event.add.__annotations__['handler']]
    
    class Handler(handler_type):
        @catch_errors
        def notify(self, args):
            callback(args)

        def __del__(self):
            log(f"Handler deleted (cb: {callback})", "debug")

    handler = Handler()
    event.add(handler)
    handler_list.append(handler)
    log(f"Handler created (cb: {callback})", "debug")


def safe_get(obj, attr, default):
    try:
        return getattr(obj, attr, default)
    except Exception:
        log(f"Failed to get {attr!r} from {obj!r}", "debug")
        return default
