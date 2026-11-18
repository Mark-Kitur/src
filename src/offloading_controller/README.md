Offloading Controller

Purpose

A ROS 2 Python package that provides the command interface for the offloading mechanism.

Communication Interface

Publisher

- Topic: "/offloading/command"
- Message type: "std_msgs/msg/Int32"

Commands

- "0" – Extend plunger
- "1" – Retract plunger

Service

- Name: "/offloading/control"
- Type: "std_srvs/srv/SetBool"
- "true" – Request extension
- "false" – Request retraction

The service translates requests into the corresponding integer commands published on the topic.