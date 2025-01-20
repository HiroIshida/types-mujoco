from ._types import *

# mj_step
# void mj_step(const mjModel* m, mjData* d);
# Advance simulation, use control callback to obtain external force and control.
def mj_step(m: MjModel, d: MjData) -> None:
    """Advance simulation, use control callback to obtain external force and control."""
    ...

# mj_step1
# void mj_step1(const mjModel* m, mjData* d);
# Advance simulation in two steps: before external force and control is set by user.
def mj_step1(m: MjModel, d: MjData) -> None:
    """Advance simulation in two steps: before external force and control is set by user."""
    ...

# mj_step2
# void mj_step2(const mjModel* m, mjData* d);
# Advance simulation in two steps: after external force and control is set by user.
def mj_step2(m: MjModel, d: MjData) -> None:
    """Advance simulation in two steps: after external force and control is set by user."""
    ...

# mj_forward
# void mj_forward(const mjModel* m, mjData* d);
# Forward dynamics: same as mj_step but do not integrate in time.
def mj_forward(m: MjModel, d: MjData) -> None:
    """Forward dynamics: same as mj_step but do not integrate in time."""
    ...

# mj_inverse
# void mj_inverse(const mjModel* m, mjData* d);
# Inverse dynamics: qacc must be set before calling.
def mj_inverse(m: MjModel, d: MjData) -> None:
    """Inverse dynamics: qacc must be set before calling."""
    ...

# mj_forwardSkip
# void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);
# Forward dynamics with skip; skipstage is mjtStage.
def mj_forwardSkip(m: MjModel, d: MjData, skipstage: int, skipsensor: int) -> None:
    """Forward dynamics with skip; skipstage is mjtStage."""
    ...

# mj_inverseSkip
# void mj_inverseSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);
# Inverse dynamics with skip; skipstage is mjtStage.
def mj_inverseSkip(m: MjModel, d: MjData, skipstage: int, skipsensor: int) -> None:
    """Inverse dynamics with skip; skipstage is mjtStage."""
    ...
