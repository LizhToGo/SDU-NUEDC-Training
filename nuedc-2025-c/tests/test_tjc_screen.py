import ast
import os
import sys
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


from tjc_screen import TJCFrame, TJCScreen  # noqa: E402


class FakeUART:
    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.writes = []

    def read(self):
        if not self.chunks:
            return None
        return self.chunks.pop(0)

    def write(self, payload):
        self.writes.append(bytes(payload))
        return len(payload)


class TJCScreenProtocolTests(unittest.TestCase):
    def test_fragmented_frame_and_resynchronization(self):
        uart = FakeUART((
            b"noise\x55\x01\x02",
            b"\x05\xff\xff\xff",
        ))
        screen = TJCScreen(uart)
        self.assertEqual(screen.poll(), [])
        frames = screen.poll()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].command, TJCScreen.CMD_START_MEASURE)
        self.assertEqual(frames[0].data0, TJCScreen.MODE_NUMBERED_SQUARE)
        self.assertEqual(frames[0].data1, 5)

    def test_rx_parser_avoids_unsupported_bytearray_deletion(self):
        source_path = os.path.join(ROOT, "tjc_screen.py")
        with open(source_path, "r", encoding="utf-8") as file:
            source = file.read()
        self.assertNotIn("del self.rx_buffer", source)

        uart = FakeUART((
            b"noise" * 100,
            b"\x55\x01\x02\x05\xff\xff\xff",
        ))
        screen = TJCScreen(uart, max_rx_buffer=32)
        self.assertEqual(screen.poll(), [])
        frames = screen.poll()
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].data1, 5)

    def test_result_scaling_and_ascii_terminators(self):
        uart = FakeUART()
        screen = TJCScreen(uart)
        screen.update_result(156.3, 10.8, 0.823, 4.12, 4.38)
        payload = uart.writes[-1]
        commands = payload.split(TJCScreen.END)
        self.assertIn(b"main.xD.val=15630", commands)
        self.assertIn(b"main.xSize.val=1080", commands)
        self.assertIn(b"main.xCurrent.val=823", commands)
        self.assertIn(b"main.xPower.val=412", commands)
        self.assertIn(b"main.xPmax.val=438", commands)
        self.assertIn(b"main.nStatus.val=2", commands)

    def test_screen_frame_maps_to_all_vision_tasks(self):
        path = os.path.join(ROOT, "single_shot_measurement.py")
        with open(path, "r", encoding="utf-8") as file:
            tree = ast.parse(file.read())
        function = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "command_from_screen_frame"
        )
        namespace = {"TJCScreen": TJCScreen}
        ast.fix_missing_locations(function)
        exec(
            compile(
                ast.Module(body=[function], type_ignores=[]),
                "<screen-map>",
                "exec",
            ),
            namespace,
        )
        convert = namespace["command_from_screen_frame"]
        self.assertEqual(
            convert(TJCFrame(1, TJCScreen.MODE_AUTO, 0)), "MEASURE"
        )
        self.assertEqual(
            convert(TJCFrame(1, TJCScreen.MODE_MIN_SQUARE, 0)),
            "ADVANCED_MIN",
        )
        self.assertEqual(
            convert(TJCFrame(1, TJCScreen.MODE_NUMBERED_SQUARE, 7)),
            "DIGIT:7",
        )
        self.assertEqual(
            convert(TJCFrame(1, TJCScreen.MODE_TILTED, 0)), "TILT_MIN"
        )

    def test_boot_explicitly_imports_main(self):
        boot_path = os.path.join(ROOT, "boot.py")
        main_path = os.path.join(ROOT, "main.py")
        self.assertTrue(os.path.isfile(boot_path))
        self.assertTrue(os.path.isfile(main_path))
        with open(boot_path, "r", encoding="utf-8") as file:
            boot_tree = ast.parse(file.read())
        with open(main_path, "r", encoding="utf-8") as file:
            main_tree = ast.parse(file.read())
        boot_imports = [
            alias.name
            for node in ast.walk(boot_tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        ]
        main_imports = [
            alias.name
            for node in ast.walk(main_tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        ]
        self.assertIn("main", boot_imports)
        self.assertIn("single_shot_measurement", main_imports)


if __name__ == "__main__":
    unittest.main()
