import ast
from pathlib import Path
import sys


class AddDocstrings(ast.NodeTransformer):
    def __init__(self):
        super().__init__()
        self.class_stack: list[str] = []

    def visit_ClassDef(self, node: ast.ClassDef):
        self.class_stack.append(node.name)
        self.generic_visit(node)
        self.class_stack.pop()
        return node

    def visit_FunctionDef(self, node: ast.FunctionDef):
        self.generic_visit(node)
        if ast.get_docstring(node, clean=False) is None:
            doc = self._make_doc(node)
            node.body.insert(0, ast.Expr(value=ast.Constant(value=doc)))
        return node

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef):
        self.generic_visit(node)
        if ast.get_docstring(node, clean=False) is None:
            doc = self._make_doc(node)
            node.body.insert(0, ast.Expr(value=ast.Constant(value=doc)))
        return node

    def _make_doc(self, node: ast.AST) -> str:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            name = node.name
            in_class = len(self.class_stack) > 0
            kind = "metoda" if in_class else "funkcija"
            qual = f"{self.class_stack[-1]}." if in_class else ""
            # prepare parameter list excluding common 'self'/'cls'
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                params = [a.arg for a in node.args.args if a.arg not in {"self", "cls"}]
            else:
                params = []
            if params:
                return f"Kratki docstring: {kind} {qual}{name}({', '.join(params)})."
            return f"Kratki docstring: {kind} {qual}{name}."
        return "Kratki docstring."


def add_docstrings_to_file(path: Path) -> None:
    src = path.read_text(encoding="utf-8", errors="replace")
    tree = ast.parse(src)
    tree = AddDocstrings().visit(tree)
    ast.fix_missing_locations(tree)
    new_src = ast.unparse(tree)
    if not new_src.endswith("\n"):
        new_src += "\n"
    path.write_text(new_src, encoding="utf-8")


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: add_docstrings.py <python_file>")
        return 2
    p = Path(sys.argv[1])
    if not p.is_file():
        print(f"Not a file: {p}")
        return 2
    add_docstrings_to_file(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

