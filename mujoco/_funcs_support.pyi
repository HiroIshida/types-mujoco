from ._types import *

# mj_stateSize
# int mj_stateSize(const mjModel* m, unsigned int spec);
# Returns the number of mjtNum⁠s required for a given state specification. The bits of the integer spec correspond to element fields of mjtState.
def mj_stateSize(m: MjModel, spec: int) -> int:
    """Returns the number of mjtNum⁠s required for a given state specification. The bits of the integer spec correspond to element fields of mjtState."""
    ...

# mj_getState
# void mj_getState(const mjModel* m, const mjData* d, mjtNum* state, unsigned int spec);
# Copy concatenated state components specified by spec from d into state. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid.
def mj_getState(m: MjModel, d: MjData, state: np.ndarray, spec: int) -> None:
    """Copy concatenated state components specified by spec from d into state. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid."""
    ...

# mj_setState
# void mj_setState(const mjModel* m, mjData* d, const mjtNum* state, unsigned int spec);
# Copy concatenated state components specified by spec from state into d. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid.
def mj_setState(m: MjModel, d: MjData, state: np.ndarray, spec: int) -> None:
    """Copy concatenated state components specified by spec from state into d. The bits of the integer spec correspond to element fields of mjtState. Fails with mju_error if spec is invalid."""
    ...

# mj_setKeyframe
# void mj_setKeyframe(mjModel* m, const mjData* d, int k);
# Copy current state to the k-th model keyframe.
def mj_setKeyframe(m: MjModel, d: MjData, k: int) -> None:
    """Copy current state to the k-th model keyframe."""
    ...

# mj_addContact
# int mj_addContact(const mjModel* m, mjData* d, const mjContact* con);
# Add contact to d->contact list; return 0 if success; 1 if buffer full.
def mj_addContact(m: MjModel, d: MjData, con: MjContact) -> int:
    """Add contact to d->contact list; return 0 if success; 1 if buffer full."""
    ...

# mj_isPyramidal
# int mj_isPyramidal(const mjModel* m);
# Determine type of friction cone.
def mj_isPyramidal(m: MjModel) -> int:
    """Determine type of friction cone."""
    ...

# mj_isSparse
# int mj_isSparse(const mjModel* m);
# Determine type of constraint Jacobian.
def mj_isSparse(m: MjModel) -> int:
    """Determine type of constraint Jacobian."""
    ...

# mj_isDual
# int mj_isDual(const mjModel* m);
# Determine type of solver (PGS is dual, CG and Newton are primal).
def mj_isDual(m: MjModel) -> int:
    """Determine type of solver (PGS is dual, CG and Newton are primal)."""
    ...

# mj_mulJacVec
# void mj_mulJacVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);
# This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space.
def mj_mulJacVec(m: MjModel, d: MjData, res: np.ndarray, vec: np.ndarray) -> None:
    """This function multiplies the constraint Jacobian mjData.efc_J by a vector. Note that the Jacobian can be either dense or sparse; the function is aware of this setting. Multiplication by J maps velocities from joint space to constraint space."""
    ...

# mj_mulJacTVec
# void mj_mulJacTVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);
# Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint space.
def mj_mulJacTVec(m: MjModel, d: MjData, res: np.ndarray, vec: np.ndarray) -> None:
    """Same as mj_mulJacVec but multiplies by the transpose of the Jacobian. This maps forces from constraint space to joint space."""
    ...

# mj_jac
# void mj_jac(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr,
#             const mjtNum point[3], int body);
# This function computes an end-effector kinematic Jacobian, describing the local linear relationship between the degrees-of-freedom and a given point. Given a body specified by its integer id (body) and a 3D point in the world frame (point) treated as attached to the body, the Jacobian has both translational (jacp) and rotational (jacr) components. Passing NULL for either pointer will skip that part of the computation. Each component is a 3-by-nv matrix. Each row of this matrix is the gradient of the corresponding coordinate of the specified point with respect to the degrees-of-freedom. The frame with respect to which the Jacobian is computed is centered at the body center-of-mass but aligned with the world frame. The minimal pipeline stages required for Jacobian computations to be consistent with the current generalized positions mjData.qpos are mj_kinematics followed by mj_comPos.
def mj_jac(
    m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, point: np.ndarray, body: int
) -> None:
    """This function computes an end-effector kinematic Jacobian, describing the local linear relationship between the degrees-of-freedom and a given point. Given a body specified by its integer id (body) and a 3D point in the world frame (point) treated as attached to the body, the Jacobian has both translational (jacp) and rotational (jacr) components. Passing NULL for either pointer will skip that part of the computation. Each component is a 3-by-nv matrix. Each row of this matrix is the gradient of the corresponding coordinate of the specified point with respect to the degrees-of-freedom. The frame with respect to which the Jacobian is computed is centered at the body center-of-mass but aligned with the world frame. The minimal pipeline stages required for Jacobian computations to be consistent with the current generalized positions mjData.qpos are mj_kinematics followed by mj_comPos."""
    ...

# mj_jacBody
# void mj_jacBody(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body);
# This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or site. They are just shortcuts; the same can be achieved by calling mj_jac directly.
def mj_jacBody(m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, body: int) -> None:
    """This and the remaining variants of the Jacobian function call mj_jac internally, with the center of the body, geom or site. They are just shortcuts; the same can be achieved by calling mj_jac directly."""
    ...

# mj_jacBodyCom
# void mj_jacBodyCom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body);
# Compute body center-of-mass end-effector Jacobian.
def mj_jacBodyCom(m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, body: int) -> None:
    """Compute body center-of-mass end-effector Jacobian."""
    ...

# mj_jacSubtreeCom
# void mj_jacSubtreeCom(const mjModel* m, mjData* d, mjtNum* jacp, int body);
# Compute subtree center-of-mass end-effector Jacobian.
def mj_jacSubtreeCom(m: MjModel, d: MjData, jacp: np.ndarray, body: int) -> None:
    """Compute subtree center-of-mass end-effector Jacobian."""
    ...

# mj_jacGeom
# void mj_jacGeom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int geom);
# Compute geom end-effector Jacobian.
def mj_jacGeom(m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, geom: int) -> None:
    """Compute geom end-effector Jacobian."""
    ...

# mj_jacSite
# void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site);
# Compute site end-effector Jacobian.
def mj_jacSite(m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, site: int) -> None:
    """Compute site end-effector Jacobian."""
    ...

# mj_jacPointAxis
# void mj_jacPointAxis(const mjModel* m, mjData* d, mjtNum* jacPoint, mjtNum* jacAxis,
#                      const mjtNum point[3], const mjtNum axis[3], int body);
# Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
def mj_jacPointAxis(
    m: MjModel,
    d: MjData,
    jacPoint: np.ndarray,
    jacAxis: np.ndarray,
    point: np.ndarray,
    axis: np.ndarray,
    body: int,
) -> None:
    """Compute translation end-effector Jacobian of point, and rotation Jacobian of axis."""
    ...

# mj_jacDot
# void mj_jacDot(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr,
#                const mjtNum point[3], int body);
# This function computes the time-derivative of an end-effector kinematic Jacobian computed by mj_jac. The minimal pipeline stages required for computation to be consistent with the current generalized positions and velocities mjData.{qpos, qvel} are mj_kinematics, mj_comPos, mj_comVel (in that order).
def mj_jacDot(
    m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, point: np.ndarray, body: int
) -> None:
    """This function computes the time-derivative of an end-effector kinematic Jacobian computed by mj_jac. The minimal pipeline stages required for computation to be consistent with the current generalized positions and velocities mjData.{qpos, qvel} are mj_kinematics, mj_comPos, mj_comVel (in that order)."""
    ...

# mj_angmomMat
# void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body);
# This function computes the 3 x nv angular momentum matrix
# H
# (
# q
# )
# H(q), providing the linear mapping from generalized velocities to subtree angular momentum. More precisely if
# h
# h is the subtree angular momentum of body index body in mjData.subtree_angmom (reported by the subtreeangmom sensor) and
# q
# ˙
# q
# ˙
# ​
#   is the generalized velocity mjData.qvel, then
# h
# =
# H
# q
# ˙
# h=H
# q
# ˙
# ​
#  .
def mj_angmomMat(m: MjModel, d: MjData, mat: np.ndarray, body: int) -> None:
    """This function computes the 3 x nv angular momentum matrix H(q), providing the linear mapping from generalized velocities to subtree angular momentum. More precisely if h is the subtree angular momentum of body index body in mjData.subtree_angmom (reported by the subtreeangmom sensor) and q˙ is the generalized velocity mjData.qvel, then h=Hq˙."""
    ...

# mj_name2id
# int mj_name2id(const mjModel* m, int type, const char* name);
# Get id of object with the specified mjtObj type and name, returns -1 if id not found.
def mj_name2id(m: MjModel, type: Union[int, mjtObj], name: str) -> int:
    """Get id of object with the specified mjtObj type and name, returns -1 if id not found."""
    ...

# mj_id2name
# const char* mj_id2name(const mjModel* m, int type, int id);
# Get name of object with the specified mjtObj type and id, returns NULL if name not found.
def mj_id2name(m: MjModel, type: Union[int, mjtObj], id: int) -> str:
    """Get name of object with the specified mjtObj type and id, returns NULL if name not found."""
    ...

# mj_fullM
# void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);
# Convert sparse inertia matrix M into full (i.e. dense) matrix.
def mj_fullM(m: MjModel, dst: np.ndarray, M: np.ndarray) -> None:
    """Convert sparse inertia matrix M into full (i.e. dense) matrix."""
    ...

# mj_mulM
# void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);
# This function multiplies the joint-space inertia matrix stored in mjData.qM by a vector. qM has a custom sparse format that the user should not attempt to manipulate directly. Alternatively one can convert qM to a dense matrix with mj_fullM and then user regular matrix-vector multiplication, but this is slower because it no longer benefits from sparsity.
def mj_mulM(m: MjModel, d: MjData, res: np.ndarray, vec: np.ndarray) -> None:
    """This function multiplies the joint-space inertia matrix stored in mjData.qM by a vector. qM has a custom sparse format that the user should not attempt to manipulate directly. Alternatively one can convert qM to a dense matrix with mj_fullM and then user regular matrix-vector multiplication, but this is slower because it no longer benefits from sparsity."""
    ...

# mj_mulM2
# void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);
# Multiply vector by (inertia matrix)^(1/2).
def mj_mulM2(m: MjModel, d: MjData, res: np.ndarray, vec: np.ndarray) -> None:
    """Multiply vector by (inertia matrix)^(1/2)."""
    ...

# mj_addM
# void mj_addM(const mjModel* m, mjData* d, mjtNum* dst, int* rownnz, int* rowadr, int* colind);
# Add inertia matrix to destination matrix. Destination can be sparse uncompressed, or dense when all int* are NULL
def mj_addM(
    m: MjModel,
    d: MjData,
    dst: np.ndarray,
    rownnz: np.ndarray,
    rowadr: np.ndarray,
    colind: np.ndarray,
) -> None:
    """Add inertia matrix to destination matrix. Destination can be sparse uncompressed, or dense when all int* are NULL"""
    ...

# mj_applyFT
# void mj_applyFT(const mjModel* m, mjData* d, const mjtNum force[3], const mjtNum torque[3],
#                 const mjtNum point[3], int body, mjtNum* qfrc_target);
# This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes we want to add the result to a different vector.
def mj_applyFT(
    m: MjModel,
    d: MjData,
    force: np.ndarray,
    torque: np.ndarray,
    point: np.ndarray,
    body: int,
    qfrc_target: np.ndarray,
) -> None:
    """This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes we want to add the result to a different vector."""
    ...

# mj_objectVelocity
# void mj_objectVelocity(const mjModel* m, const mjData* d,
#                        int objtype, int objid, mjtNum res[6], int flg_local);
# Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
def mj_objectVelocity(
    m: MjModel, d: MjData, objtype: int, objid: int, res: np.ndarray, flg_local: int
) -> None:
    """Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation."""
    ...

# mj_objectAcceleration
# void mj_objectAcceleration(const mjModel* m, const mjData* d,
#                            int objtype, int objid, mjtNum res[6], int flg_local);
# Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation. If acceleration or force sensors are not present in the model, mj_rnePostConstraint must be manually called in order to calculate mjData.cacc – the total body acceleration, including contributions from the constraint solver.
def mj_objectAcceleration(
    m: MjModel, d: MjData, objtype: int, objid: int, res: np.ndarray, flg_local: int
) -> None:
    """Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation. If acceleration or force sensors are not present in the model, mj_rnePostConstraint must be manually called in order to calculate mjData.cacc – the total body acceleration, including contributions from the constraint solver."""
    ...

# mj_geomDistance
# mjtNum mj_geomDistance(const mjModel* m, const mjData* d, int geom1, int geom2,
#                        mjtNum distmax, mjtNum fromto[6]);
# Returns the smallest signed distance between two geoms and optionally the segment from geom1 to geom2. Returned distances are bounded from above by distmax.
# If no collision of distance smaller than distmax is found, the function will return distmax and fromto, if given, will be set to (0, 0, 0, 0, 0, 0).
#
# Positive distmax values
#
# For some colliders, a large, positive distmax will result in an accurate measurement. However, for collision pairs which use the general mjc_Convex collider, the result will be approximate and likely innacurate. This is considered a bug to be fixed in a future release. In order to determine whether a geom pair uses mjc_Convex, inspect the table at the top of engine_collision_driver.c.
def mj_geomDistance(
    m: MjModel, d: MjData, geom1: int, geom2: int, distmax: float, fromto: np.ndarray
) -> float:
    """Returns the smallest signed distance between two geoms and optionally the segment from geom1 to geom2. Returned distances are bounded from above by distmax. If no collision of distance smaller than distmax is found, the function will return distmax and fromto, if given, will be set to (0, 0, 0, 0, 0, 0). Positive distmax values For some colliders, a large, positive distmax will result in an accurate measurement. However, for collision pairs which use the general mjc_Convex collider, the result will be approximate and likely innacurate. This is considered a bug to be fixed in a future release. In order to determine whether a geom pair uses mjc_Convex, inspect the table at the top of engine_collision_driver.c."""

# mj_contactForce
# void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]);
# Extract 6D force:torque given contact id, in the contact frame.
def mj_contactForce(m: MjModel, d: MjData, id: int, result: np.ndarray) -> None:
    """Extract 6D force:torque given contact id, in the contact frame."""
    ...

# mj_differentiatePos
# void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
#                          const mjtNum* qpos1, const mjtNum* qpos2);
# This function subtracts two vectors in the format of qpos (and divides the result by dt), while respecting the properties of quaternions. Recall that unit quaternions represent spatial orientations. They are points on the unit sphere in 4D. The tangent to that sphere is a 3D plane of rotational velocities. Thus when we subtract two quaternions in the right way, the result is a 3D vector and not a 4D vector. Thus the output qvel has dimensionality nv while the inputs have dimensionality nq.
def mj_differentiatePos(
    m: MjModel, qvel: np.ndarray, dt: float, qpos1: np.ndarray, qpos2: np.ndarray
) -> None:
    """This function subtracts two vectors in the format of qpos (and divides the result by dt), while respecting the properties of quaternions. Recall that unit quaternions represent spatial orientations. They are points on the unit sphere in 4D. The tangent to that sphere is a 3D plane of rotational velocities. Thus when we subtract two quaternions in the right way, the result is a 3D vector and not a 4D vector. Thus the output qvel has dimensionality nv while the inputs have dimensionality nq."""

# mj_integratePos
# void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);
# This is the opposite of mj_differentiatePos. It adds a vector in the format of qvel (scaled by dt) to a vector in the format of qpos.
def mj_integratePos(m: MjModel, qpos: np.ndarray, qvel: np.ndarray, dt: float) -> None:
    """This is the opposite of mj_differentiatePos. It adds a vector in the format of qvel (scaled by dt) to a vector in the format of qpos."""
    ...

# mj_normalizeQuat
# void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);
# Normalize all quaternions in qpos-type vector.
def mj_normalizeQuat(m: MjModel, qpos: np.ndarray) -> None:
    """Normalize all quaternions in qpos-type vector."""
    ...

# mj_local2Global
# void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9], const mjtNum pos[3],
#                      const mjtNum quat[4], int body, mjtByte sameframe);
# Map from body local to global Cartesian coordinates, sameframe takes values from mjtSameFrame.
def mj_local2Global(
    d: MjData,
    xpos: np.ndarray,
    xmat: np.ndarray,
    pos: np.ndarray,
    quat: np.ndarray,
    body: int,
    sameframe: int,
) -> None:
    """Map from body local to global Cartesian coordinates, sameframe takes values from mjtSameFrame."""
    ...

# mj_getTotalmass
# mjtNum mj_getTotalmass(const mjModel* m);
# Sum all body masses.
def mj_getTotalmass(m: MjModel) -> float:
    """Sum all body masses."""
    ...

# mj_setTotalmass
# void mj_setTotalmass(mjModel* m, mjtNum newmass);
# Scale body masses and inertias to achieve specified total mass.
def mj_setTotalmass(m: MjModel, newmass: float) -> None:
    """Scale body masses and inertias to achieve specified total mass."""
    ...

# mj_getPluginConfig
# const char* mj_getPluginConfig(const mjModel* m, int plugin_id, const char* attrib);
# Return a config attribute value of a plugin instance; NULL: invalid plugin instance ID or attribute name
def mj_getPluginConfig(m: MjModel, plugin_id: int, attrib: str) -> str:
    """Return a config attribute value of a plugin instance; NULL: invalid plugin instance ID or attribute name"""
    ...

# mj_loadPluginLibrary
# void mj_loadPluginLibrary(const char* path);
# Load a dynamic library. The dynamic library is assumed to register one or more plugins.
def mj_loadPluginLibrary(path: str) -> None:
    """Load a dynamic library. The dynamic library is assumed to register one or more plugins."""
    ...

# mj_loadAllPluginLibraries
# void mj_loadAllPluginLibraries(const char* directory, mjfPluginLibraryLoadCallback callback);
# Scan a directory and load all dynamic libraries. Dynamic libraries in the specified directory are assumed to register one or more plugins. Optionally, if a callback is specified, it is called for each dynamic library encountered that registers plugins.
def mj_loadAllPluginLibraries(directory: str, callback: Callable) -> None:
    """Scan a directory and load all dynamic libraries. Dynamic libraries in the specified directory are assumed to register one or more plugins. Optionally, if a callback is specified, it is called for each dynamic library encountered that registers plugins."""
    ...

# mj_version
# int mj_version(void);
# Return version number: 1.0.2 is encoded as 102.
def mj_version() -> int:
    """Return version number: 1.0.2 is encoded as 102."""
    ...

# mj_versionString
# const char* mj_versionString(void);
# Return the current version of MuJoCo as a null-terminated string.
def mj_versionString() -> str:
    """Return the current version of MuJoCo as a null-terminated string."""
    ...
