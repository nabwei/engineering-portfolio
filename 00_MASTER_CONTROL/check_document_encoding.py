"""Reject invalid UTF-8 and known double-encoding signatures; a UTF-8 BOM is valid."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRECTORIES = ('00_MASTER_CONTROL', '01_FPGA_EMBEDDED', '02_ROBOTICS_NAVIGATION',
                      '03_COMPUTER_VISION', '04_SENSOR_FUSION', '05_MEASUREMENT_TIMING',
                      '06_UAV_DIGITAL_TWIN', '07_STUDY_GUIDES')
MARKERS = ('\u0637§\u0638„', 'â€', 'â‰', 'â†', '\ufffd')
EXCLUDED = {'datasets', 'build', 'install', 'log', '__pycache__'}


def document_problems(path):
    try:
        text = path.read_text(encoding='utf-8-sig')
    except UnicodeDecodeError:
        return [{'line': None, 'reason': 'invalid UTF-8'}]
    return [{'line': number, 'reason': 'known double-encoding or replacement signature'}
            for number, line in enumerate(text.splitlines(), 1) if any(marker in line for marker in MARKERS)]


def source_documents(root):
    yield from root.glob('*.md')
    for folder in SOURCE_DIRECTORIES:
        for path in (root / folder).rglob('*.md'):
            parts = path.relative_to(root).parts
            if not EXCLUDED.intersection(parts) and not any(part.startswith(('build-', 'build_')) for part in parts):
                yield path


def main():
    documents = sorted(source_documents(ROOT))
    problems = [{'file': path.relative_to(ROOT).as_posix(), **problem}
                for path in documents for problem in document_problems(path)]
    print(json.dumps({'status': 'FAIL' if problems else 'PASS', 'documents': len(documents),
                      'scope': 'UTF-8 and known corruption signatures, not semantic proofreading',
                      'problems': problems}, ensure_ascii=False, indent=2))
    return bool(problems)


if __name__ == '__main__':
    raise SystemExit(main())
