"""Running the fake gamepad inside somebody else's rclpy context.

An integration test harness generally keeps a context of its own, so that the system under test
cannot wedge the test's own node - `better_launch_testing` does, and the fake gamepad is meant
to be driven from exactly there:

    gamepad = env.attach_node(FakeJoyPublisher(context=env.context))
    gamepad.drive(1.0)

Two things have to hold for that to work, and neither is visible from a standalone run: the node
has to be built in the given context, and the parameter fetch in its constructor has to be
spun by an executor belonging to that context rather than by rclpy's global one - which belongs
to the default context and, in a process that never called `rclpy.init()`, does not exist.

Plain pytest on purpose. `test_fake_joy_publisher.py` is a launch_testing file and needs a real
manager; nothing here does.
"""

import threading

import pytest
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from hector_gamepad_testing_tools.fake_joy_publisher import (
    _find_manager_fqn,
    _request_params_sync,
)


@pytest.fixture
def private_ros():
    """A context of its own, spinning, with no global context anywhere in the process."""
    context = Context()
    context.init()

    executor = MultiThreadedExecutor(context=context)
    spinning = threading.Thread(target=executor.spin, daemon=True)
    spinning.start()

    try:
        yield context, executor
    finally:
        executor.shutdown()
        context.try_shutdown()


class TestAgainstAPrivateContext:
    def test_parameters_can_be_fetched(self, private_ros):
        """The constructor asks the manager for `config_name` before it can do anything at all,
        so this failing means the fake gamepad cannot be built in a harness at all."""
        context, executor = private_ros

        manager = Node("pretend_manager", context=context)
        manager.declare_parameter("config_name", "athena")
        executor.add_node(manager)

        caller = Node("caller", context=context)
        found = _request_params_sync(caller, "/pretend_manager", ("config_name",))

        assert found == {"config_name": "athena"}

    def test_the_client_is_not_left_behind(self, private_ros):
        """A fake gamepad built once per test would otherwise leak one service client per
        test, on a node that lives for the whole file."""
        context, executor = private_ros

        manager = Node("pretend_manager", context=context)
        manager.declare_parameter("config_name", "athena")
        executor.add_node(manager)

        caller = Node("caller", context=context)
        _request_params_sync(caller, "/pretend_manager", ("config_name",))

        assert list(caller.clients) == []

    def test_a_failed_fetch_does_not_leak_either(self, private_ros):
        """The path that is taken when the manager is not up yet, which is the common way for
        this to be called wrongly."""
        context, _ = private_ros
        caller = Node("caller", context=context)

        with pytest.raises(RuntimeError):
            _request_params_sync(caller, "/nobody_here", ("config_name",))

        assert list(caller.clients) == []

    def test_the_graph_is_visible_from_the_private_context(self, private_ros):
        """Node discovery has to see the harness' domain, or the manager is never found."""
        context, _ = private_ros
        caller = Node("caller", context=context)

        with pytest.raises(RuntimeError) as failure:
            _find_manager_fqn(caller, "hector_gamepad_manager", timeout_sec=0.3)

        assert "/caller" in str(failure.value), (
            "the diagnostic should list what it did see, and it should see this context"
        )
