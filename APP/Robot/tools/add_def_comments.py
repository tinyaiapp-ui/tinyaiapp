import ast
from pathlib import Path
import sys
import re


class ParentAnnotator(ast.NodeVisitor):
    def __init__(self):
        self.parents = {}

    def visit(self, node):
        for child in ast.iter_child_nodes(node):
            self.parents[child] = node
        super().visit(node)


def is_method(node: ast.AST, parents: dict) -> bool:
    p = parents.get(node)
    return isinstance(p, ast.ClassDef)


def describe_function(name: str) -> str:
    n = name.lower()
    parts = []
    # Primary purpose
    if 'ik' in n:
        if '2' in n and 'dof' in n:
            parts.append('inverzna kinematika (2-DOF) za pomicanje')
        else:
            parts.append('inverzna kinematika za pomicanje')
    elif 'fk' in n:
        parts.append('naprijedna kinematika i izračun položaja')
    elif 'tcp' in n:
        parts.append('rad s TCP-om (pomicanje/računanje vrha alata)')
    elif 'send' in n and 'pos' in n:
        parts.append('slanje pozicija motorima preko serijske veze')
    elif 'serial' in n or 'ser' == n or 'port' in n:
        parts.append('upravljanje serijskom vezom i portovima')
    elif 'draw' in n or 'plot' in n or 'render' in n:
        parts.append('crtanje/prikaz robota ili UI elemenata')
    elif 'update' in n:
        parts.append('ažuriranje stanja i korisničkog sučelja')
    elif 'load' in n or 'read' in n:
        parts.append('učitavanje podataka ili konfiguracije')
    elif 'save' in n or 'write' in n or 'store' in n:
        parts.append('spremanje podataka ili konfiguracije')
    elif 'create' in n or 'build' in n or 'init' in n:
        parts.append('inicijalizacija i priprema potrebnih resursa')
    elif 'home' in n:
        parts.append('povratak ili postavljanje početnog položaja')
    elif 'grip' in n:
        parts.append('upravljanje hvataljkom')
    elif 'joystick' in n:
        parts.append('obrada ulaza joysticka')
    elif 'limit' in n or 'bound' in n:
        parts.append('provjera i namještanje granica/limita')
    elif 'deg' in n or 'rad' in n or 'angle' in n:
        parts.append('pretvorbe i računanje kuteva')
    elif n in {'_t', '_rot', 'rot', 't'} or 'mat' in n or 'rot' in n:
        parts.append('matrične transformacije (translacija/rotacija)')
    elif 'log' in n or 'print' in n:
        parts.append('logiranje i ispis poruka')
    elif 'schedule' in n or 'timer' in n:
        parts.append('planiranje odgođenih akcija/ponovnog crteža')
    elif 'cli' in n or 'arg' in n:
        parts.append('obrada argumenata komandne linije')
    elif 'connect' in n or 'open' in n:
        parts.append('uspostava veze s uređajem/portom')
    elif 'disconnect' in n or 'close' in n:
        parts.append('prekid veze i čišćenje resursa')
    else:
        parts.append('pomoćna logika specifična za aplikaciju')

    extra = []
    if 'pos' in n or 'position' in n:
        extra.append('rad s pozicijama')
    if 'speed' in n or 'vel' in n:
        extra.append('upravljanje brzinom')
    if 'accel' in n or 'acc' in n:
        extra.append('upravljanje ubrzanjem')
    if 'file' in n or 'json' in n or 'seq' in n:
        extra.append('rad s datotekama/sekvencama')

    base = parts[0]
    if extra:
        base += ' i ' + ', '.join(extra)
    return base


def insert_comments(src: str) -> str:
    tree = ast.parse(src)
    annot = ParentAnnotator()
    annot.visit(tree)

    # Collect function defs with locations
    functions: list[tuple[int, str, bool]] = []  # (lineno, name, is_method)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            functions.append((node.lineno, node.name, is_method(node, annot.parents)))

    # Sort by line number
    functions.sort(key=lambda t: t[0])

    lines = src.splitlines()
    offset = 0
    for lineno, name, in_class in functions:
        idx = lineno - 1 + offset
        if idx < 0 or idx >= len(lines):
            continue
        line = lines[idx]
        # Ensure this line looks like a def (to be robust after edits)
        m = re.match(r"^(?P<indent>\s*)def\s+\w+\s*\(", line)
        if not m:
            # Try to search forward a few lines in case of decorators
            search_idx = idx
            for j in range(idx, min(idx + 5, len(lines))):
                mm = re.match(r"^(?P<indent>\s*)def\s+\w+\s*\(", lines[j])
                if mm:
                    m = mm
                    idx = j
                    break
        if not m:
            continue
        indent = m.group('indent')

        role = 'metodi' if in_class else 'funkciji'
        desc = describe_function(name)
        comment = f"{indent}# U ovoj {role} je {desc}."
        # Avoid duplicating if a similar comment exists just above, or any comment directly above
        prev_line = lines[idx-1] if idx-1 >= 0 else ""
        check_start = max(0, idx - 3)
        recent = "\n".join(lines[check_start:idx])
        if re.search(r"^\s*#\s+U ovoj (metodi|funkciji) je", recent, flags=re.M):
            continue
        if re.match(r"^\s*#", prev_line):
            # Some comment already directly above this def; skip
            continue
        # Insert a blank line then the comment before the def
        lines.insert(idx, comment)
        lines.insert(idx, "")
        offset += 2
    # Collapse consecutive duplicate comments of the same content
    deduped = []
    prev = None
    for ln in lines:
        if prev is not None and ln == prev and re.match(r"^\s*#\s+U ovoj (metodi|funkciji) je", ln):
            # skip duplicate
            continue
        deduped.append(ln)
        prev = ln
    return "\n".join(deduped) + ("\n" if not src.endswith("\n") else "")


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: add_def_comments.py <python_file>")
        return 2
    p = Path(sys.argv[1])
    if not p.is_file():
        print(f"Not a file: {p}")
        return 2
    src = p.read_text(encoding='utf-8', errors='replace')
    new_src = insert_comments(src)
    p.write_text(new_src, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
