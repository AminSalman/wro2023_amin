# SPDX-License-Identifier: MIT
# Copyright (c) 2018-2020 The Pybricks Authors

"""
Classes to exchange messages between EV3 bricks.
"""

from __future__ import annotations

from typing import abstractmethod, TypeVar, Optional, Callable, Generic

T = TypeVar("T")


class Connection:
    @abstractmethod
    def read_from_mailbox(self, name: str) -> bytes:
        ...

    @abstractmethod
    def send_to_mailbox(self, name: str, data: bytes) -> None:
        ...

    @abstractmethod
    def wait_for_mailbox_update(self, name: str) -> None:
        ...


class Mailbox(Generic[T]):
    def __init__(
        self,
        name: str,
        connection: Connection,
        encode: Optional[Callable[[T], bytes]] = None,
        decode: Optional[Callable[[bytes], T]] = None,
    ):
        """Mailbox(name, connection, encode=None, decode=None)

        Object that represents a mailbox containing data.

        You can read data that is delivered by other EV3 bricks, or send data
        to other bricks that have the same mailbox.

        By default, the mailbox reads and send only bytes. To send other
        data, you can provide an ``encode`` function that encodes your Python
        object into bytes, and a ``decode`` function to convert bytes back to
        a Python object.

        Arguments:
            name (str):
                The name of this mailbox.
            connection:
                A connection object such as :class:`BluetoothMailboxClient`.
            encode (callable):
                Function that encodes a Python object to bytes.
            decode (callable):
                Function that creates a new Python object from bytes.
        """

    def read(self) -> T:
        """read()

        Gets the current value of the mailbox.

        Returns:
            The current value or ``None`` if the mailbox is empty.
        """
        return ""

    def send(self, value: T, brick: Optional[str] = None) -> None:
        """send(value, brick=None)

        Sends a value to this mailbox on connected devices.

        Arguments:
            value:
                The value that will be delivered to the mailbox.
            brick (str):
                The name or Bluetooth address of the brick or ``None`` to
                to broadcast to all connected devices.

        Raises:
            OSError:
                There is a problem with the connection.
        """

    def wait(self) -> None:
        """wait()

        Waits for the mailbox to be updated by remote device."""

    def wait_new(self) -> T:
        """wait_new()

        Waits for a new value to be delivered to the mailbox that is not
        equal to the current value in the mailbox.

        Returns:
            The new value.
        """
        return object()


class LogicMailbox(Mailbox[bool]):
    def __init__(self, name: str, connection: Connection):
        """LogicMailbox(name, connection)

        Object that represents a mailbox containing boolean data.

        This works just like a regular :class:`Mailbox`, but values
        must be ``True`` or ``False``.

        This is compatible with the "logic" mailbox type in EV3-G.

        Arguments:
            name (str):
                The name of this mailbox.
            connection:
                A connection object such as :class:`BluetoothMailboxClient`.
        """


class NumericMailbox(Mailbox[float]):
    def __init__(self, name: str, connection: Connection):
        """NumericMailbox(name, connection)

        Object that represents a mailbox containing numeric data.

        This works just like a regular :class:`Mailbox`, but values must be a
        number, such as ``15`` or ``12.345``

        This is compatible with the "numeric" mailbox type in EV3-G.

        Arguments:
            name (str):
                The name of this mailbox.
            connection:
                A connection object such as :class:`BluetoothMailboxClient`.
        """


class TextMailbox(Mailbox[str]):
    def __init__(self, name: str, connection: Connection):
        """TextMailbox(name, connection)

        Object that represents a mailbox containing text data.

        This works just like a regular :class:`Mailbox`, but data must be a
        string, such as ``'hello!'``.

        This is compatible with the "text" mailbox type in EV3-G.

        Arguments:
            name (str):
                The name of this mailbox.
            connection:
                A connection object such as :class:`BluetoothMailboxClient`.
        """


class BluetoothMailboxServer:
    """Object that represents a Bluetooth connection from one or more remote
    EV3s.

    The remote EV3s can either be running MicroPython or the standard EV3
    firmware.

    A "server" waits for a "client" to connect to it.
    """

    def __enter__(self) -> BluetoothMailboxServer:
        return self

    def __exit__(self, type, value, traceback) -> None:
        self.server_close()

    def wait_for_connection(self, count: int = 1) -> None:
        """wait_for_connection(count=1)

        Waits for a :class:`BluetoothMailboxClient` on a remote device to
        connect.

        Arguments:
            count (int):
                The number of remote connections to wait for.

        Raises:
            OSError:
                There was a problem establishing the connection.
        """

    def server_close(self) -> None:
        """server_close()

        Closes all connections."""


class BluetoothMailboxClient:
    """Object that represents a Bluetooth connection to one or more remote EV3s.

    The remote EV3s can either be running MicroPython or the standard EV3
    firmware.

    A "client" initiates a connection to a waiting "server".
    """

    def __enter__(self) -> BluetoothMailboxClient:
        return self

    def __exit__(self, type, value, traceback) -> None:
        self.close()

    def connect(self, brick: str) -> None:
        """connect(brick)

        Connects to an :class:`BluetoothMailboxServer` on another device.

        The remote device must be paired and waiting for a connection. See
        :meth:`BluetoothMailboxServer.wait_for_connection`.

        Arguments:
            brick (str):
                The name or Bluetooth address of the remote EV3 to connect to.

        Raises:
            OSError:
                There was a problem establishing the connection.
        """

    def close(self) -> None:
        """close()

        Closes all connections."""
