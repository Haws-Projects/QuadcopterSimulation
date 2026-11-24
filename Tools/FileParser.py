# dat_reader.py
from pathlib import Path


class FileParser:
    def __init__(self, filepath):
        self.values = {}
        self.rows = []

        for raw_line in Path(filepath).read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if not line:
                continue

            if "=" in line:
                key, val = [part.strip() for part in line.split("=", 1)]
                self.values[key] = val.strip("'\"")
            else:
                parts = line.replace(",", " ").split()
                try:
                    self.rows.append([float(x) for x in parts])
                except ValueError:
                    pass

    def get(self, key, default=None):
        if key not in self.values:
            return default
        val = self.values[key]
        if val.lower() == "true":
            return True
        if val.lower() == "false":
            return False
        try:
            return int(val)
        except ValueError:
            pass
        try:
            return float(val)
        except ValueError:
            return val

    def get_list(self, key, default=None):
        if key not in self.values:
            return default
        return [float(x) for x in self.values[key].replace(",", " ").split()]

    def get_rows(self):
        return self.rows