# AICA Applications

An AICA application is interconnected graph of **components** and **hardware interfaces**. Components process data in a
periodic step function and transfer data as signals to other components and hardware interfaces. Hardware interfaces
are drivers that connect controllers to robots. The application state can be dynamically changed through **events**,
which can change parameters, load or unload components, trigger specific service actions and more.

The nodes and edges of an AICA application graph can be visualized diagrammatically. It is also possible to define the
graph structure in a text format using YAML syntax.

## Validating a YAML Application

The [YAML application schema](./schema/application.schema.json) defines the structural rules of an
AICA application and effectively distinguishes between valid and invalid syntax.

## Migration guides

### Migrating from 1-4-x to 2-0-0

#### General property names

Previously, capital letters were permitted in component, hardware, controller, condition and sequence names. Now, all
names in the application must be `lower_snake_case`.

#### New required properties

The new schema requires two new top-level properties to be defined in all applications. These are `schema`, which must
declare the syntax version of the current application, and `dependencies`, which must at minimum define the version of
the AICA Core image required to run the application. Refer to the complete documentation for additional information
on these new properties.

```yaml
schema: 2-0-0

dependencies:
  core: v4.0.0
```

#### Graph positions

Application elements (components, hardware, on_start and buttons) previously had a `position` property to define their
placement on the visual application graph. In the new syntax, graph information has been moved to the `graph` property,
with positions specifically defined under a `positions` sub-property.

This allows for more compact and portable definitions of application logic, separate from visual information.

In addition, position coordinates must now be a multiple of 20 to align with the grid spacing.

Before:

```yaml
on_start:
  # ...
  position:
    x: -115  # coordinates must be a multiple of 20 in the new syntax version
    y: 0

components:
  my_component:
    component: ...
    position:
      x: 100
      y: 200

hardware:
  my_hardware:
    urdf: ...
    rate: ...
    position:
      x: 200
      y: 500
```

After:

```yaml
on_start:
# ...

components:
  my_component:
    component: ...

hardware:
  my_hardware:
    urdf: ...
    rate: ...

graph:
  positions:
    # the position of on_start is defined at this level
    on_start:
      x: -120
      y: 0
    # the positions of components, hardware, conditions, sequences and buttons are defined under each property,
    # and are matched with the element by name
    components:
      my_component:
        x: 100
        y: 200
    hardware:
      my_hardware:
        x: 200
        y: 500
```

#### Events on start

The `on_start` property is now only permitted to load components, load hardware and start sequences. Previously, it was
possible to trigger lifecycle transitions, switch controllers or call services from the `on_start` event, in addition
to potentially restarting or aborting a sequence.

The restrictive change is intended to prevent non-deterministic behavior, for example when trying to load a controller
on a hardware interface that has not yet been loaded, or triggering a lifecycle transition on a component that has not
yet been loaded.

Migration of now unsupported `on_start` events from the old syntax can be done in two ways: either use the `on_load`
state transition event trigger of components or hardware directly to propagate further events, or move the events into a
sequence and start the sequence instead.

In the example below, the start events have been moved to a sequence. Additionally, intermediate sequence check steps
are added to ensure the expected state before proceeding. While the new syntax requirement is more verbose, it allows
for more deterministic application starts including error handling if any start events fail.

Before:

```yaml
on_start:
  load:
    - component: my_component
    - hardware: my_hardware
    - controller: my_controller
      hardware: my_hardware
  lifecycle:
    component: my_component
    transition: configure
  switch_controllers:
    hardware: my_hardware
    activate: my_controller
```

After:

```yaml
on_start:
  load:
    - component: my_component
    - hardware: my_hardware
  sequence:
    start: my_sequence

sequences:
  my_sequence:
    steps:
      - check:
          condition:
            component: my_component
            state: loaded
          wait_forever: true
      - lifecycle:
          component: my_component
          transition: configure
      - check:
          condition:
            hardware: my_hardware
            state: active
          wait_forever: true
      - load:
          controller: my_controller
          hardware: my_hardware
      - check:
          condition:
            controller: my_controller
            hardware: my_hardware
            state: loaded
          wait_forever: true
      - switch_controllers:
          hardware: my_hardware
          activate: my_controller
```

#### Predicate and transition events

Previously, components could only trigger events through predicates. These were listed directly under the `events`
field:

```yaml
components:
  my_component:
    component: my_package::Foo
    events:
      is_inactive:
        load: foo
      is_in_range:
        unload: foo
```

Now, predicates are explicitly listed under a `predicates` subfield. 2-0-0 also introduces state transitions as event
triggers.

```yaml
components:
  my_component:
    component: my_package::Foo
    events:
      transitions:
        on_configure:
          load: foo
      predicates:
        is_in_range:
          unload: foo
```

The same concept applies equally to controllers:

```yaml
controllers:
  my_controller:
    plugin: my_package::Foo
    events:
      transitions:
        on_activate:
          load: foo
      predicates:
        is_in_range:
          unload: foo
```

#### Self-targeting events

Previously, certain component predicate-driven events could be implicitly self-targeting; the target component did not
need to be specified in the event object if the target component was the same as the component triggering the event.
This applied to lifecycle transition, service call and parameter setting events.

Now, the target component is always required as part of the event structure for these events. This is because events
can be triggered from many sources, including predicate or state transitions from components, controllers, or hardware,
or as a result of sequence steps or conditions; implicitly determining the target component from the event source is
not possible in the majority of these cases.

Before:

```yaml
lifecycle: activate
call_service:
  service: foo
set:
  parameter: foo
  value: bar
```

After:

```yaml
lifecycle:
  component: foo
  transition: activate
call_service:
  component: foo
  service: foo
set:
  component: foo
  parameter: foo
  value: bar
```

#### Component mapping

The `mapping` property of components has been renamed to `remapping`. The pattern restrictions for remapped names and
namespaces have been revised to allow all legal options in ROS 2.

Before:

```yaml
components:
  my_component:
    component: my_package::Foo
    mapping:
      name: foo
      namespace: my_namespace
```

After:

```yaml
components:
  my_component:
    component: my_package::Foo
    remapping:
      name: foo
      namespace: my_namespace
```

#### Sequences

Sequences now require their steps to be listed under the `steps` field.

Before:

```yaml
sequences:
  my_sequence:
    - load:
        component: my_component
    - switch_controllers:
        hardware: my_hardware
        activate: my_controller
```

After:

```yaml
sequences:
  my_sequence:
    steps:
      - load:
          component: my_component
      - switch_controllers:
          hardware: my_hardware
          activate: my_controller
```

Additionally, the special sequence steps of `wait` and `assert` have been replaced with `delay` and `check`.

Previously, the `wait` step was used to wait for either a fixed time interval or for a condition or predicate to be
true. Now, the `delay` step is used to wait for fixed time interval in seconds, while the `check` step can be used to
wait for a condition or predicate.

When migrating from the conditional `wait` sequence step to the new `check` step, the `timeout: { seconds: T }`
object simply becomes a duration in seconds as `timeout: T`. Similarly, the sub-property `timeout: {events: ... }` for
breakout events after timeout instead becomes `else: ...`.

If a conditional wait step should wait forever with no timeout, it previously required the `wait: { timeout: ... }`
property to be left undefined. Now, the `check` step instead has an explicit property `wait_forever: true`.

Conditional steps must now also define their condition source under the `condition` sub-property of `check`, rather than
directly under the `wait` or `assert` steps.

Just as before, if a conditional wait defines a timeout, then breakout events will be handled and the sequence will be
aborted if the condition is not true within the defined time.

Before:

```yaml
my_sequence:
  # case 1: fixed time wait
  - wait:
      seconds: 10
  # case 2a: wait for condition (wait forever with no timeout)
  - wait:
      condition: my_condition
  # case 2b: wait for predicate (wait forever with no timeout)
  - wait:
      component: foo
      predicate: bar
  # case 3a: wait for condition (with timeout and optional breakout events after timeout)
  - wait:
      condition: my_condition
      timeout:
        seconds: 10
        events:
          unload:
            component: foo
  # case 3b: wait for condition (with timeout and optional breakout events after timeout)
  - wait:
      component: foo
      predicate: bar
      timeout:
        seconds: 10
        events:
          unload:
            component: foo
```

After:

```yaml
my_sequence:
  steps:
    # case 1: fixed time wait
    - delay: 10
    # case 2a: wait for condition (wait forever with no timeout)
    - check:
        condition:
          condition: my_condition
        wait_forever: true
    # case 2b: wait for predicate (wait forever with no timeout)
    - check:
        condition:
          component: foo
          predicate: bar
        wait_forever: true
    # case 3a: wait for condition (with timeout and optional breakout events after timeout)
    - check:
        condition:
          condition: my_condition
        timeout: 10
        else:
          unload:
            component: foo
    # case 3b: wait for condition (with timeout and optional breakout events after timeout)
    - check:
        condition:
          component: foo
          predicate: bar
        timeout: 10
        else:
          unload:
            component: foo
```

Previously, the `assert` step was used to conditionally check a condition or predicate and immediately abort the
sequence if the condition was false, triggering any defined breakout events on the way. The new `check` step functions
in the same way if no timeout is defined.

Just as before, if a conditional wait defines a timeout, then breakout events will be handled and the sequence will be
aborted if the condition is not true within the defined time.

Before:

```yaml
my_sequence:
  # case 1a: check a condition and abort the sequence immediately on failure (with optional breakout events)
  - assert:
      condition: my_condition
      else:
        unload:
          component: my_component
  # case 1b: check a predicate and abort the sequence immediately on failure (with optional breakout events)
  - assert:
      component: foo
      predicate: bar
      else:
        unload:
          component: my_component
```

After:

```yaml
my_sequence:
  steps:
    # case 1a: check a condition and abort the sequence immediately on failure (with optional breakout events)
    - check:
        condition:
          condition: my_condition
        else:
          unload:
            component: my_component
    # case 1b: check a predicate and abort the sequence immediately on failure (with optional breakout events)
    - check:
        condition:
          component: foo
          predicate: bar
        else:
          unload:
            component: my_component
```

Finally, looping sequences should now be expressed with the `loop: true` property instead of explicitly restarting the
current sequence as the last step.

Before:

```yaml
my_sequence:
  - wait:
      seconds: 10
  - load:
      component: foo
  - wait:
      seconds: 10
  - unload:
      component: foo
  - sequence:
      restart: my_sequence
```

After:

```yaml
my_sequence:
  loop: true
  steps:
    - delay: 10
    - load:
        component: foo
    - delay: 10
    - unload:
        component: foo
```

#### Conditions

Conditions now require the actual condition source including any conditional operators to be moved under the
`condition` property instead of existing on the top level alongside the optional condition events.

Before:

```yaml
conditions:
  # case 1: example component predicate condition source that triggers no events
  condition_1:
    component: my_component
    predicate: foo

  # case 2: example controller predicate condition source that also triggers an event
  condition_2:
    controller: my_controller
    hardware: my_hardware
    predicate: bar
    events:
      load:
        component: foo

  # case 3: conditional operator on multiple sources that also triggers an event
  condition_3:
    any:
      - { component: my_component, predicate: foo }
      - { controller: my_controller, hardware: my_hardware,  predicate: foo }
    events:
      load:
        component: foo
```

After:

```yaml
conditions:
  # case 1: example component predicate condition source that triggers no events
  condition_1:
    condition:
      component: my_component
      predicate: foo

  # case 2: example controller predicate condition source that also triggers an event
  condition_2:
    condition:
      controller: my_controller
      hardware: my_hardware
      predicate: bar
    events:
      load:
        component: foo

  # case 3: conditional operator on multiple sources that also triggers an event
  condition_3:
    condition:
      any:
        - { component: my_component, predicate: foo }
        - { controller: my_controller, hardware: my_hardware,  predicate: foo }
    events:
      load:
        component: foo
```
