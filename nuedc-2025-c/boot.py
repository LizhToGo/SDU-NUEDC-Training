"""CanMV boot entry that explicitly starts ``main.py``.

This Yahboom/CanMV image reliably executes ``/sdcard/boot.py`` but does not
reliably chain to ``main.py`` by itself.  Therefore boot imports main
explicitly.  Create ``/data/disable_autostart`` before reboot when an IDE-only
maintenance session is required; remove it for competition deployment.
"""

import os
import sys


try:
    from ybUtils.YbRGB import YbRGB

    YbRGB().show_rgb((0, 0, 0))
except Exception as error:
    print("BOOT_RGB_ERROR:", repr(error))


def _autostart_disabled():
    try:
        os.stat("/data/disable_autostart")
        return True
    except OSError:
        return False


if _autostart_disabled():
    print("AUTOSTART_DISABLED: remove /data/disable_autostart to run main.py")
else:
    try:
        # CanMV normally puts /sdcard on sys.path, but this is not consistent
        # across firmware revisions and IDE/MTP boot modes.  Make the path
        # explicit before importing the production entry point.
        if "/sdcard" not in sys.path:
            sys.path.append("/sdcard")
        import main  # noqa: F401
    except ImportError as error:
        # A few images execute boot.py with an empty module search path.  The
        # direct source fallback keeps the device self-starting without
        # relying on a particular CanMV launcher implementation.
        try:
            with open("/sdcard/main.py", "r") as main_file:
                source = main_file.read()
            namespace = {"__name__": "__main__", "__file__": "/sdcard/main.py"}
            exec(source, namespace)
        except Exception:
            print("AUTOSTART_IMPORT_ERROR:", repr(error))
            raise
    except Exception as error:
        print("AUTOSTART_ERROR:", repr(error))
        raise
