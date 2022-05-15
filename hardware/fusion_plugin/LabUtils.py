# Extend pythonpath to be able to import "labutils" package
import sys, os
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from labutils.common import *
from labutils.commands import BaseCommand


def setup_toolbar():
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    tab = workspace.toolbarTabs.itemById(MAIN_TAB_ID)
    if tab is None:
        raise RuntimeError(f"Main tab not found (id: {MAIN_TAB_ID})")
    panel = tab.toolbarPanels.itemById(MAIN_PANEL_ID)
    if panel is None:
        panel = tab.toolbarPanels.add(MAIN_PANEL_ID, MAIN_PANEL_NAME)


def clear_toolbar():
    workspace = ui.workspaces.itemById(WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(MAIN_PANEL_ID)
    if panel is not None:
        for cmd in BaseCommand.__instances__:
            delete_command(cmd.id)
        panel.deleteMe()
        log("Deleted main toolbar panel", "debug")


@catch_errors
def run(_):
    log("Starting...", "debug")
    setup_toolbar()
    BaseCommand.register_all()
    log("*** LabUtils plugin activated ***", "warning")


@catch_errors
def stop(_):
    log("Stopping...", "debug")
    clear_toolbar()
    BaseCommand.__handlers__ = []
    log("*** LabUtils plugin deactivated ***", "warning")
    del sys.modules["labutils.common"]
    del sys.modules["labutils.commands"]
