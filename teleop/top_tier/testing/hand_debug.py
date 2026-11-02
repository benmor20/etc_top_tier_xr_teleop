

"""Parse Python-style object repr strings, e.g. MyClass(a=1, b=[Sub(x="y")]), into dicts."""

import ast
import json
import re
import math

_IDENT = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*")
_NUMBER = re.compile(
    r"[+-]?(?:inf|nan|(?:\d[\d_]*\.?[\d_]*|\.\d[\d_]*)(?:[eE][+-]?\d+)?)"
)
_STRING_PREFIX = re.compile(r"[rRbBuU]{0,2}(?=['\"])")
_KEYWORDS = {"True": True, "False": False, "None": None}


class ReprParseError(ValueError):
    pass


def parse_repr(text):
    """Parse a repr string into nested dicts.

    Objects become dicts with a `_class` key plus one key per keyword argument.
    Supports int, float, str, bool, None, list, tuple, dict, and nested objects.
    """
    parser = _Parser(text)
    parser.skip_ws()
    result = parser.parse_value()
    parser.skip_ws()
    if parser.pos != len(text):
        parser.fail("unexpected trailing content")
    return result


class _Parser:
    def __init__(self, text):
        self.s = text
        self.pos = 0

    def fail(self, msg):
        snippet = self.s[max(0, self.pos - 15): self.pos + 15]
        raise ReprParseError(f"{msg} at position {self.pos}: ...{snippet!r}...")

    def skip_ws(self):
        while self.pos < len(self.s) and self.s[self.pos].isspace():
            self.pos += 1

    def peek(self):
        return self.s[self.pos] if self.pos < len(self.s) else ""

    def expect(self, ch):
        self.skip_ws()
        if self.peek() != ch:
            self.fail(f"expected {ch!r}")
        self.pos += 1

    def parse_value(self):
        self.skip_ws()
        c = self.peek()
        if not c:
            self.fail("unexpected end of input")
        if c == "[":
            return self.parse_sequence("[", "]")
        if c == "(":
            return tuple(self.parse_sequence("(", ")"))
        if c == "{":
            return self.parse_brace()
        if _STRING_PREFIX.match(self.s, self.pos):
            return self.parse_string()
        m = _IDENT.match(self.s, self.pos)
        if m and m.group() not in ("inf", "nan"):
            return self.parse_ident_or_object(m)
        m = _NUMBER.match(self.s, self.pos)
        if m:
            self.pos = m.end()
            text = m.group().replace("_", "")
            try:
                return int(text)
            except ValueError:
                return float(text)
        self.fail("could not parse value")

    def parse_string(self):
        start = self.pos
        m = _STRING_PREFIX.match(self.s, self.pos)
        self.pos = m.end()
        quote = self.s[self.pos]
        if self.s.startswith(quote * 3, self.pos):
            quote *= 3
        self.pos += len(quote)
        while self.pos < len(self.s):
            if self.s[self.pos] == "\\":
                self.pos += 2
            elif self.s.startswith(quote, self.pos):
                self.pos += len(quote)
                return ast.literal_eval(self.s[start:self.pos])
            else:
                self.pos += 1
        self.fail("unterminated string")

    def parse_ident_or_object(self, m):
        name = m.group()
        self.pos = m.end()
        if name in _KEYWORDS:
            return _KEYWORDS[name]
        self.skip_ws()
        if self.peek() != "(":
            self.fail(f"unexpected bare identifier {name!r}")
        self.pos += 1
        obj = {"_class": name}
        while True:
            self.skip_ws()
            if self.peek() == ")":
                self.pos += 1
                return obj
            key = _IDENT.match(self.s, self.pos)
            if not key:
                self.fail("expected property name")
            self.pos = key.end()
            self.expect("=")
            obj[key.group()] = self.parse_value()
            self.skip_ws()
            if self.peek() == ",":
                self.pos += 1
            elif self.peek() != ")":
                self.fail("expected ',' or ')'")

    def parse_sequence(self, open_ch, close_ch):
        self.expect(open_ch)
        items = []
        while True:
            self.skip_ws()
            if self.peek() == close_ch:
                self.pos += 1
                return items
            items.append(self.parse_value())
            self.skip_ws()
            if self.peek() == ",":
                self.pos += 1
            elif self.peek() != close_ch:
                self.fail(f"expected ',' or {close_ch!r}")

    def parse_brace(self):
        self.expect("{")
        d = {}
        items = []
        is_set = False
        while True:
            self.skip_ws()
            if self.peek() == "}":
                self.pos += 1
                return items if is_set else d
            k = self.parse_value()
            self.skip_ws()
            if self.peek() == ":":
                self.pos += 1
                d[k] = self.parse_value()
            else:
                is_set = True
                items.append(k)
            self.skip_ws()
            if self.peek() == ",":
                self.pos += 1
            elif self.peek() != "}":
                self.fail("expected ',' or '}'")


MISSING = object()  # sentinel for keys present on only one side


def diff(a, b, path="", rel_tol=0.0, abs_tol=0.0):
    """Return {path: (a_value, b_value)} for every leaf that differs.

    Paths look like "child.items[2].name". Subobjects and lists are recursed
    into, so only the leaves that actually changed are reported. A key present
    on only one side is reported with MISSING on the other side. Floats can be
    compared with a tolerance via rel_tol / abs_tol.
    """
    out = {}
    _diff(a, b, path, out, rel_tol, abs_tol)
    return out


def _diff(a, b, path, out, rel_tol, abs_tol):
    if isinstance(a, dict) and isinstance(b, dict):
        for key in list(a) + [k for k in b if k not in a]:
            sub = f"{path}.{key}" if path else str(key)
            if key not in a or key not in b:
                out[sub] = (a.get(key, MISSING), b.get(key, MISSING))
            else:
                _diff(a[key], b[key], sub, out, rel_tol, abs_tol)
    elif isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        if len(a) != len(b):
            out[path] = (a, b)  # shouldn't happen per your assumption
            return
        for i, (x, y) in enumerate(zip(a, b)):
            _diff(x, y, f"{path}[{i}]", out, rel_tol, abs_tol)
    elif not _equal(a, b, rel_tol, abs_tol):
        out[path] = (a, b)


def _equal(a, b, rel_tol, abs_tol):
    # bool is a subclass of int, so True == 1; treat differing types as different
    if isinstance(a, bool) != isinstance(b, bool):
        return False
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return True
        return math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)
    return a == b


def format_diff(d):
    """Pretty-print the result of diff()."""
    show = lambda v: "<missing>" if v is MISSING else repr(v)
    return "\n".join(f"{p}: {show(x)} -> {show(y)}" for p, (x, y) in d.items())


def clean_line(log: str) -> str:
    return " ".join(line.strip() for line in log.split())


def main():
    original_log = """left_msg=HandCmd_(motor_cmd=[MotorCmd_(mode=16, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0),
                         MotorCmd_(mode=17, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=18, q=0,                               
                         dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=19, q=0, dq=0.0, tau=0.0, kp=1.5,                              
                         kd=0.2, reserve=0), MotorCmd_(mode=20, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0),                                   
                         MotorCmd_(mode=21, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=22, q=0,                               
                         dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0)], reserve=[0, 0, 0, 0]),                                                       
                         right_msg=HandCmd_(motor_cmd=[MotorCmd_(mode=16, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2,                                     
                         reserve=0), MotorCmd_(mode=17, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0),                                           
                         MotorCmd_(mode=18, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=19, q=0,                               
                         dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=20, q=0, dq=0.0, tau=0.0, kp=1.5,                              
                         kd=0.2, reserve=0), MotorCmd_(mode=21, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0),                                   
                         MotorCmd_(mode=22, q=0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0)], reserve=[0, 0, 0, 0])"""
    our_log = """left_msg=HandCmd_(motor_cmd=[MotorCmd_(mode=16, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=17, q=-0.7, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=18, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=19, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=20, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=21, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=22, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0)], reserve=[0, 0, 0, 0]),
right_msg=HandCmd_(motor_cmd=[MotorCmd_(mode=16, q=0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=17, q=0.7, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=18, q=-0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=19, q=-0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=20, q=-0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=21, q=-0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0), MotorCmd_(mode=22, q=-0.0, dq=0.0, tau=0.0, kp=1.5, kd=0.2, reserve=0)], reserve=[0, 0, 0, 0])
""".strip()

    original_log = clean_line(original_log)
    our_log = clean_line(our_log)

    og_left_log, og_right_log = [log[log.index("=")+1:] for log in original_log.split(", right_ms")]
    our_left_log, our_right_log = [log[log.index("=")+1:] for log in our_log.split(", right_ms")]
    og_left = parse_repr(og_left_log)
    og_right = parse_repr(og_right_log)
    our_left = parse_repr(our_left_log)
    our_right = parse_repr(our_right_log)

    print("LEFT DIFF")
    left_diff = diff(og_left, our_left)
    print(format_diff(left_diff))

    print("RIGHT DIFF")
    right_diff = diff(og_right, our_right)
    print(format_diff(right_diff))


if __name__ == '__main__':
    main()
