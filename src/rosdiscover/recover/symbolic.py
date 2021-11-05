# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "Concatenate",
    "StringLiteral",
    "SymbolicArg",
    "SymbolicAssignment",
    "SymbolicBool",
    "SymbolicCompound",
    "SymbolicFunction",
    "SymbolicNodeHandle",
    "SymbolicNodeHandleImpl",
    "SymbolicNodeName",
    "SymbolicParameter",
    "SymbolicProgram",
    "SymbolicStatement",
    "SymbolicString",
    "SymbolicUnknown",
)

import abc
import enum
import typing
import typing as t

from loguru import logger
import attr

from ..interpreter import NodeContext


@attr.s(auto_attribs=True, slots=True)
class SymbolicContext:
    """Used to maintain program state during interpretation.

    Attributes
    ----------
    program: SymbolicProgram
        The program that is being interpreted.
    function: SymbolicFunction
        The current function that is being interpreted.
    node: NodeContext
        The node context that the symbolic program is being used to construct.
    _vars: t.Dict[str, t.Any]
        A mapping from the names of in-scope variables to their values.
    """
    program: SymbolicProgram
    function: SymbolicFunction
    node: NodeContext
    _vars: t.Dict[str, t.Any] = attr.ib(factory=dict)

    @classmethod
    def create(
        cls,
        program: SymbolicProgram,
        node: NodeContext,
    ) -> SymbolicContext:
        return SymbolicContext(
            program=program,
            function=program.entrypoint,
            node=node,
        )

    def for_function_call(
        self,
        function: SymbolicFunction,
        args: t.Optional[t.Mapping[str, t.Any]] = None,
    ) -> SymbolicContext:
        """Creates a new symbolic context that represents the scope of a function call."""
        context = SymbolicContext(self.program, function, self.node)
        args = args or {}
        for arg_name, arg_val in args.items():
            context.store(arg_name, arg_val)
        logger.debug(f"creating symbolic context for function call: {context}")
        return context

    def load(self, variable: str) -> t.Any:
        """Loads the value of a given variable.

        Raises
        ------
        ValueError
            If no variable exists with the given name.
        """
        logger.debug(f"attempting to read value of variable: {variable}")
        return self._vars[variable]

    def store(self, variable: str, value: t.Any) -> None:
        """Stores the value of a given variable.

        Raises
        ------
        ValueError
            If a definition for the given variable already exists as that would violate
            single static assignment (SSA) rules.
        """
        if variable in self._vars:
            raise ValueError(f"variable already defined in this scope: {variable}")

        self._vars[variable] = value
        logger.debug(f"stored symbolic variable [{variable}] value: {value}")

    @property
    def node_name(self) -> str:
        """Returns the name of the associated node."""
        return self.node.name


class SymbolicValueType(enum.Enum):
    BOOL = "bool"
    INTEGER = "integer"
    NODE_HANDLE = "node-handle"
    STRING = "string"
    UNSUPPORTED = "unsupported"

    def __str__(self) -> str:
        return self.value

    @classmethod
    def from_name(cls, name: str) -> SymbolicValueType:
        name_to_type = {
            "bool": cls.BOOL,
            "integer": cls.INTEGER,
            "node-handle": cls.NODE_HANDLE,
            "string": cls.STRING,
            "unsupported": cls.UNSUPPORTED,
        }
        if name not in name_to_type:
            raise ValueError(f"unknown value type: {name}")
        return name_to_type[name]


class SymbolicValue(abc.ABC):
    """Represents a symbolic value in a function summary."""
    @abc.abstractmethod
    def to_dict(self) -> t.Dict[str, t.Any]:
        ...

    @abc.abstractmethod
    def eval(self, context: SymbolicContext) -> t.Any:
        ...

    @abc.abstractmethod
    def is_unknown(self) -> bool:
        """Returns whether or not this symbolic value is unknown."""
        ...


class SymbolicString(SymbolicValue, abc.ABC):
    """Represents a symbolic string value."""


class SymbolicNodeName(SymbolicString):
    """Symbolically refers to the name of the current node."""
    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"kind": "node-name"}

    def eval(self, context: SymbolicContext) -> t.Any:
        return context.node_name

    def is_unknown(self) -> bool:
        return False


@attr.s(frozen=True, auto_attribs=True, slots=True)
class StringLiteral(SymbolicString):
    """Represents a literal string value."""
    value: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "string-literal",
            "literal": self.value,
        }

    def eval(self, context: SymbolicContext) -> t.Any:
        return self.value

    def is_unknown(self) -> bool:
        return False


@attr.s(frozen=True, auto_attribs=True, slots=True)
class Concatenate(SymbolicString):
    """Represents a concatenation of two symbolic strings."""
    lhs: SymbolicString
    rhs: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "concatenate",
            "lhs": self.lhs.to_dict(),
            "rhs": self.rhs.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> t.Any:
        lhs = self.lhs.eval(context)
        rhs = self.rhs.eval(context)
        if isinstance(lhs, str) and isinstance(rhs, str):
            return lhs + rhs
        else:
            return SymbolicUnknown()

    def is_unknown(self) -> bool:
        return self.lhs.is_unknown() or self.rhs.is_unknown()


class SymbolicInteger(SymbolicValue, abc.ABC):
    """Represents a symbolic integer value."""


class SymbolicBool(SymbolicValue, abc.ABC):
    """Represents a symbolic boolean value."""


class SymbolicNodeHandle(SymbolicString, SymbolicValue, abc.ABC):
    """Represents a symbolic node handle."""


class SymbolicUnknown(
    SymbolicNodeHandle,
    SymbolicInteger,
    SymbolicBool,
    SymbolicString,
    SymbolicValue,
):
    """Represents an unknown symbolic value."""
    # TODO: this will cause issues with saving summaries to JSON/YAML
    def eval(self, context: SymbolicContext) -> t.Any:
        return self

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"kind": "unknown"}

    def is_unknown(self) -> bool:
        return True


# FIXME this is the effect of a bad class hierarchy :-(
# I'll fix this up later
@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicNodeHandleImpl(SymbolicNodeHandle):
    namespace: SymbolicString

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "node-handle",
            "namespace": self.namespace.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> t.Any:
        return self.namespace.eval(context)

    def is_unknown(self) -> bool:
        return self.namespace.is_unknown()


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicArg(
    SymbolicNodeHandle,
    SymbolicInteger,
    SymbolicBool,
    SymbolicString,
    SymbolicValue,
):
    name: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "arg",
            "name": self.name,
        }

    def eval(self, context: SymbolicContext) -> t.Any:
        return context.load(self.name)

    def is_unknown(self) -> bool:
        return False


class SymbolicStatement(abc.ABC):
    """Represents a statement in a symbolic function summary."""
    @abc.abstractmethod
    def to_dict(self) -> t.Dict[str, t.Any]:
        ...

    @abc.abstractmethod
    def eval(self, context: SymbolicContext) -> None:
        ...


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicAssignment(SymbolicStatement):
    """Represents an assignment to a symbolic variable.

    Attributes
    ----------
    variable: str
        The name of the variable whose value is being assigned.
    value: SymbolicValue
        The value of the variable, provided as a symbolic expression.
    """
    variable: str
    value: SymbolicValue

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "assignment",
            "variable": self.variable,
            "value": self.value.to_dict(),
        }

    def eval(self, context: SymbolicContext) -> None:
        concrete_value = self.value.eval(context)
        context.store(self.variable, concrete_value)


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicCompound(t.Sequence[SymbolicStatement], SymbolicStatement):
    """Represents a sequence of symbolic statements."""
    _statements: t.Sequence[SymbolicStatement] = attr.ib(factory=list)

    def __len__(self) -> int:
        return len(self._statements)

    @typing.overload
    def __getitem__(self, at: int) -> SymbolicStatement:
        ...

    @typing.overload
    def __getitem__(self, at: slice) -> t.Sequence[SymbolicStatement]:
        ...

    def __getitem__(
        self,
        at: t.Union[int, slice],
    ) -> t.Union[SymbolicStatement, t.Sequence[SymbolicStatement]]:
        return self._statements[at]

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "compound",
            "statements": [s.to_dict() for s in self._statements],
        }

    def eval(self, context: SymbolicContext) -> None:
        for statement in self._statements:
            statement.eval(context)


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicFunctionCall(SymbolicStatement):
    """Represents a call to a symbolic function.

    Attributes
    ----------
    callee: str
        The name of the function that is being called.
    arguments: t.Mapping[str, SymbolicValue]
        The arguments that should be used during the function call, indexed by
        the name of the corresponding argument.
    """
    callee: str
    arguments: t.Mapping[str, SymbolicValue]

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "call",
            "callee": self.callee,
            "arguments": {
                name: arg.to_dict() for (name, arg) in self.arguments.items()
            },
        }

    def eval(self, context: SymbolicContext) -> None:
        args: t.Dict[str, t.Any] = {}
        for arg_name, arg_symbolic_value in self.arguments.items():
            args[arg_name] = arg_symbolic_value.eval(context)
            logger.debug(f"obtained symbolic value for argument [{arg_name}]: {args[arg_name]}")

        function = context.program.functions[self.callee]
        context = context.for_function_call(function, args)
        function.body.eval(context)


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicVariableReference(SymbolicValue):
    """Represents a symbolic variable reference.

    Attributes
    ----------
    variable: str
        The name of the value that is referenced.
    type: SymbolicValueType
        The type of the referenced variable.
    """
    variable: str
    type_: SymbolicValueType

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "kind": "variable-reference",
            "variable": self.variable,
            "type": str(self.type_),
        }

    def eval(self, context: SymbolicContext) -> t.Any:
        return context.load(self.variable)

    def is_unknown(self) -> bool:
        """Warning: We do not check whether the definition of the variable is unknown."""
        return False


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicParameter:
    """Provides the definition for a symbolic function parameter."""
    index: int
    name: str
    type_: SymbolicValueType

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "index": self.index,
            "name": self.name,
            "type": str(self.type_),
        }


@attr.s(frozen=True, auto_attribs=True, slots=True)
class SymbolicFunction:
    """Provides the definition of a single symbolic function.

    Attributes
    ----------
    name: str
        The fully-qualified name of the function.
    parameters: t.Mapping[str, SymbolicParameter]
        The parameters of this function, indexed by name.
    body: SymbolicCompound
        The definition of the function.
    """
    name: str
    parameters: t.Mapping[str, SymbolicParameter] = attr.ib(hash=False)
    body: SymbolicCompound = attr.ib(hash=False)

    @classmethod
    def empty(cls, name: str) -> SymbolicFunction:
        """Creates an empty function with a given name that takes no arguments."""
        return cls.build(name, [], SymbolicCompound())

    @classmethod
    def build(
        cls,
        name: str,
        parameters: t.Iterable[SymbolicParameter],
        body: SymbolicCompound,
    ) -> SymbolicFunction:
        name_to_parameter = {param.name: param for param in parameters}
        return SymbolicFunction(name, name_to_parameter, body)

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "name": self.name,
            "parameters": [p.to_dict() for p in self.parameters.values()],
            "body": self.body.to_dict(),
        }

    @property
    def calls(self) -> t.Set[str]:
        """Returns the names of functions that are called within this function."""
        return set(stmt.callee for stmt in self.body if isinstance(stmt, SymbolicFunctionCall))


@attr.s(frozen=True, slots=True)
class SymbolicProgram:
    """Provides a symbolic summary for a given program.

    Attributes
    ----------
    entrypoint_name: str
        The name of the function that serves as the entry point for the
        program.
    functions: t.Mapping[str, SymbolicFunction]
        The symbolic functions within this program, indexed by name.

    Raises
    ------
    ValueError
        If this program does not provide a "main" function.
    """
    entrypoint_name: str = attr.ib()
    functions: t.Mapping[str, SymbolicFunction] = attr.ib()

    @functions.validator
    def must_have_entry_function(
        self,
        attribute: str,
        value: t.Any,
    ) -> None:
        if self.entrypoint_name not in self.functions:
            raise ValueError(f"failed to find definition for entrypoint function: {self.entrypoint_name}")

    @classmethod
    def build(cls, entrypoint: str, functions: t.Iterable[SymbolicFunction]) -> SymbolicProgram:
        name_to_function = {function.name: function for function in functions}
        if entrypoint not in name_to_function:
            logger.warning(
                f"The entrypoint '{entrypoint}' does not appear to reach any ROS API calls."
                " Adding an empty placeholder function."
            )
            name_to_function[entrypoint] = SymbolicFunction.empty(entrypoint)
        return SymbolicProgram(entrypoint, name_to_function)

    @property
    def unreachable_functions(self) -> t.Set[SymbolicFunction]:
        """Returns the set of functions that are unreachable from the entrypoint of this program.
        Unreachable functions almost always indicate incomplete control flow information
        (due to, e.g., certain callbacks). In some cases, an architecturally relevant function may
        truly be unreachable.
        """
        queue: t.List[SymbolicFunction] = [self.entrypoint]
        reached: t.Set[SymbolicFunction] = set()

        while queue:
            function = queue.pop(0)
            reached.add(function)
            calls = set(self.functions[name] for name in function.calls)
            for called_function in calls:
                if called_function not in reached:
                    queue.append(called_function)

        unreachable = set(self.functions.values()).difference(reached)
        return unreachable

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "program": {
                "entrypoint": self.entrypoint_name,
                "functions": [f.to_dict() for f in self.functions.values()],
            },
        }

    @property
    def entrypoint(self) -> SymbolicFunction:
        """The entrypoint for this program."""
        return self.functions[self.entrypoint_name]

    def eval(self, node: NodeContext) -> None:
        context = SymbolicContext.create(self, node)
        self.entrypoint.body.eval(context)
