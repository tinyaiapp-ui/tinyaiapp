import io
import sys
import tokenize
import ast
from pathlib import Path


def strip_comments_and_blank_lines(source: str) -> str:
    # Prefer AST round-trip and explicitly drop docstrings and standalone string literals
    try:
        tree = ast.parse(source)

        class StripDocstrings(ast.NodeTransformer):
            def _strip_leading_doc(self, body):
                if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, (ast.Str, ast.Constant)) and isinstance(getattr(body[0], 'value', None).value, str):
                    return body[1:]
                return body

            def visit_Module(self, node: ast.Module):
                self.generic_visit(node)
                node.body = self._strip_leading_doc(node.body)
                node.body = [stmt for stmt in node.body if not (isinstance(stmt, ast.Expr) and isinstance(stmt.value, (ast.Str, ast.Constant)) and isinstance(getattr(stmt, 'value', None).value, str))]
                return node

            def visit_FunctionDef(self, node: ast.FunctionDef):
                self.generic_visit(node)
                node.body = self._strip_leading_doc(node.body)
                node.body = [stmt for stmt in node.body if not (isinstance(stmt, ast.Expr) and isinstance(stmt.value, (ast.Str, ast.Constant)) and isinstance(getattr(stmt, 'value', None).value, str))]
                return node

            def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef):
                self.generic_visit(node)
                node.body = self._strip_leading_doc(node.body)
                node.body = [stmt for stmt in node.body if not (isinstance(stmt, ast.Expr) and isinstance(stmt.value, (ast.Str, ast.Constant)) and isinstance(getattr(stmt, 'value', None).value, str))]
                return node

            def visit_ClassDef(self, node: ast.ClassDef):
                self.generic_visit(node)
                node.body = self._strip_leading_doc(node.body)
                node.body = [stmt for stmt in node.body if not (isinstance(stmt, ast.Expr) and isinstance(stmt.value, (ast.Str, ast.Constant)) and isinstance(getattr(stmt, 'value', None).value, str))]
                return node

        tree = StripDocstrings().visit(tree)
        ast.fix_missing_locations(tree)
        rebuilt = ast.unparse(tree)
        if not rebuilt.endswith("\n"):
            rebuilt += "\n"
        cleaned_lines = [ln.rstrip() for ln in rebuilt.splitlines() if ln.strip()]
        return "\n".join(cleaned_lines) + "\n"
    except Exception:
        pass
    # Tokenize and drop comments while preserving code and strings
    out_tokens = []
    try:
        # tokenize requires bytes
        byts = source.encode('utf-8')
        tokgen = tokenize.tokenize(io.BytesIO(byts).readline)
        for tok in tokgen:
            ttype = tok.type
            if ttype == tokenize.COMMENT:
                # Drop comments entirely (both full-line and inline)
                continue
            # Keep NL tokens so line structure is preserved; we'll drop empty lines later
            out_tokens.append(tok)
    except tokenize.TokenError:
        # Fallback: if tokenization fails, return original source unmodified
        return source

    # Reconstruct code
    rebuilt = tokenize.untokenize(out_tokens).decode('utf-8', errors='replace')

    # Remove lines that are empty or whitespace-only
    cleaned_lines = []
    for line in rebuilt.splitlines():
        s = line.strip()
        # Drop empty lines and stray backslash-only lines that untokenize can introduce
        if s == "" or s == "\\":
            continue
        cleaned_lines.append(line.rstrip())

    return "\n".join(cleaned_lines) + "\n"


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: strip_py_comments.py <python_file>")
        return 2
    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"Not a file: {path}")
        return 2
    src = path.read_text(encoding='utf-8', errors='replace')
    cleaned = strip_comments_and_blank_lines(src)
    path.write_text(cleaned, encoding='utf-8')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
