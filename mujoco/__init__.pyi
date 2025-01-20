import numpy as np
from typing import List, Any
from enum import Enum
# https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjmodel

class MjVisual: ...
class MjStatistic: ...

class MjWarningStat:
    lastinfo: int  """info from last warning"""
    number: int  """how many times was warning raised"""

class MjTimerStat:
    duration: float  """cumulative duration"""
    number: int  """how many times was timer called"""

class MjSolverStat:
    """per-iteration solver statistics"""
    improvement: float  """cost reduction, scaled by 1/trace(M(qpos0))"""
    gradient: float  """gradient norm (primal only, scaled)"""
    lineslope: float  """slope in linesearch"""
    nactive: int  """number of active constraints"""
    nchange: int  """number of constraint state changes"""
    neval: int  """number of cost evaluations in line search"""
    nupdate: int  """number of Cholesky updates in line search"""

class mjtObj(Enum):
    mjOBJ_UNKNOWN = 0
    mjOBJ_BODY = 1
    mjOBJ_XBODY = 2
    mjOBJ_JOINT = 3
    mjOBJ_DOF = 4
    mjOBJ_GEOM = 5
    mjOBJ_SITE = 6
    mjOBJ_CAMERA = 7
    mjOBJ_LIGHT = 8
    mjOBJ_FLEX = 9
    mjOBJ_MESH = 10
    mjOBJ_SKIN = 11
    mjOBJ_HFIELD = 12
    mjOBJ_TEXTURE = 13
    mjOBJ_MATERIAL = 14
    mjOBJ_PAIR = 15
    mjOBJ_EXCLUDE = 16
    mjOBJ_EQUALITY = 17
    mjOBJ_TENDON = 18
    mjOBJ_ACTUATOR = 19
    mjOBJ_SENSOR = 20
    mjOBJ_NUMERIC = 21
    mjOBJ_TEXT = 22
    mjOBJ_TUPLE = 23
    mjOBJ_KEY = 24
    mjOBJ_PLUGIN = 25
    mjNOBJECT = 26
    mjOBJ_FRAME = 100

class MjOption:
  timestep: float                """timestep"""
  apirate: float                 """update rate for remote API (Hz)"""

  impratio: float                """ratio of friction-to-normal contact impedance"""
  tolerance: float               """main solver tolerance"""
  ls_tolerance: float            """CG/Newton linesearch tolerance"""
  noslip_tolerance: float        """noslip solver tolerance"""
  ccd_tolerance: float           """convex collision solver tolerance"""

  gravity: np.ndarray              """gravitational acceleration size=(3,)"""
  wind: np.ndarray                 """wind (for lift, drag and viscosity) size=(3,)"""
  magnetic: np.ndarray             """global magnetic flux size=(3,)"""
  density: float                 """ density of medium"""
  viscosity: float               """ viscosity of medium"""

  o_margin: float                """margin"""
  o_solref: np.ndarray        """solref size=(mjNREF,)"""
  o_solimp: np.ndarray        """solimp size=(mjNIMPi,)"""
  o_friction: np.ndarray           """friction size=(5,)"""

  integrator: int                 """integration mode (mjtIntegrator)"""
  cone: int                       """type of friction cone (mjtCone)"""
  jacobian: int                   """type of Jacobian (mjtJacobian)"""
  solver: int                     """solver algorithm (mjtSolver)"""
  iterations: int                 """maximum number of main solver iterations"""
  ls_iterations: int              """maximum number of CG/Newton linesearch iterations"""
  noslip_iterations: int          """maximum number of noslip solver iterations"""
  ccd_iterations: int             """maximum number of convex collision solver iterations"""
  disableflags: int               """bit flags for disabling standard features"""
  enableflags: int                """bit flags for enabling optional features"""
  disableactuator: int            """bit flags for disabling actuators by group id"""

  sdf_initpoints: int             """number of starting points for gradient descent"""
  sdf_iterations: int             """max number of iterations for gradient descent"""
};


class MjModel:

  @classmethod
  def from_xml_string(cls, xml_string: str) -> 'MjModel': ...


  # sizes needed at mjModel construction
  nq: int                         """number of generalized coordinates = dim(qpos)"""
  nv: int                         """number of degrees of freedom = dim(qvel)"""
  nu: int                         """number of actuators/controls = dim(ctrl)"""
  na: int                         """number of activation states = dim(act)"""
  nbody: int                      """number of bodies"""
  nbvh: int                       """number of total bounding volumes in all bodies"""
  nbvhstatic: int                 """number of static bounding volumes (aabb stored in mjModel)"""
  nbvhdynamic: int                """number of dynamic bounding volumes (aabb stored in mjData)"""
  njnt: int                       """number of joints"""
  ngeom: int                      """number of geoms"""
  nsite: int                      """number of sites"""
  ncam: int                       """number of cameras"""
  nlight: int                     """number of lights"""
  nflex: int                      """number of flexes"""
  nflexvert: int                  """number of vertices in all flexes"""
  nflexedge: int                  """number of edges in all flexes"""
  nflexelem: int                  """number of elements in all flexes"""
  nflexelemdata: int              """number of element vertex ids in all flexes"""
  nflexelemedge: int              """number of element edge ids in all flexes"""
  nflexshelldata: int             """number of shell fragment vertex ids in all flexes"""
  nflexevpair: int                """number of element-vertex pairs in all flexes"""
  nflextexcoord: int              """number of vertices with texture coordinates"""
  nmesh: int                      """number of meshes"""
  nmeshvert: int                  """number of vertices in all meshes"""
  nmeshnormal: int                """number of normals in all meshes"""
  nmeshtexcoord: int              """number of texcoords in all meshes"""
  nmeshface: int                  """number of triangular faces in all meshes"""
  nmeshgraph: int                 """number of ints in mesh auxiliary data"""
  nskin: int                      """number of skins"""
  nskinvert: int                  """number of vertices in all skins"""
  nskintexvert: int               """number of vertiex with texcoords in all skins"""
  nskinface: int                  """number of triangular faces in all skins"""
  nskinbone: int                  """number of bones in all skins"""
  nskinbonevert: int              """number of vertices in all skin bones"""
  nhfield: int                    """number of heightfields"""
  nhfielddata: int                """number of data points in all heightfields"""
  ntex: int                       """number of textures"""
  ntexdata: int                   """number of bytes in texture rgb data"""
  nmat: int                       """number of materials"""
  npair: int                      """number of predefined geom pairs"""
  nexclude: int                   """number of excluded geom pairs"""
  neq: int                        """number of equality constraints"""
  ntendon: int                    """number of tendons"""
  nwrap: int                      """number of wrap objects in all tendon paths"""
  nsensor: int                    """number of sensors"""
  nnumeric: int                   """number of numeric custom fields"""
  nnumericdata: int               """number of mjtNums in all numeric fields"""
  ntext: int                      """number of text custom fields"""
  ntextdata: int                  """number of mjtBytes in all text fields"""
  ntuple: int                     """number of tuple custom fields"""
  ntupledata: int                 """number of objects in all tuple fields"""
  nkey: int                       """number of keyframes"""
  nmocap: int                     """number of mocap bodies"""
  nplugin: int                    """number of plugin instances"""
  npluginattr: int                """number of chars in all plugin config attributes"""
  nuser_body: int                 """number of mjtNums in body_user"""
  nuser_jnt: int                  """number of mjtNums in jnt_user"""
  nuser_geom: int                 """number of mjtNums in geom_user"""
  nuser_site: int                 """number of mjtNums in site_user"""
  nuser_cam: int                  """number of mjtNums in cam_user"""
  nuser_tendon: int               """number of mjtNums in tendon_user"""
  nuser_actuator: int             """number of mjtNums in actuator_user"""
  nuser_sensor: int               """number of mjtNums in sensor_user"""
  nnames: int                     """number of chars in all names"""
  npaths: int                     """number of chars in all paths"""

  # sizes set after mjModel construction
  nnames_map: int                 """number of slots in the names hash map"""
  nM: int                         """number of non-zeros in sparse inertia matrix"""
  nB: int                         """number of non-zeros in sparse body-dof matrix"""
  nC: int                         """number of non-zeros in sparse reduced dof-dof matrix"""
  nD: int                         """number of non-zeros in sparse dof-dof matrix"""
  nJmom: int                      """number of non-zeros in sparse actuator_moment matrix"""
  ntree: int                      """number of kinematic trees under world body"""
  ngravcomp: int                  """number of bodies with nonzero gravcomp"""
  nemax: int                      """number of potential equality-constraint rows"""
  njmax: int                      """number of available rows in constraint Jacobian (legacy)"""
  nconmax: int                    """number of potential contacts in contact list (legacy)"""
  nuserdata: int                  """number of mjtNums reserved for the user"""
  nsensordata: int                """number of mjtNums in sensor data vector"""
  npluginstate: int               """number of mjtNums in plugin state vector"""
  narena: int                     """number of bytes in the mjData arena (inclusive of stack)"""
  nbuffer: int                    """number of bytes in buffer"""

  # options and statistics
  opt: MjOption                   """physics options"""
  vis: MjVisual                   """visualization options"""
  stat: MjStatistic               """model statistics"""

  #buffers
  buffer: Any                     """main buffer; all pointers point in it    (void*)"""

  # default generalized coordinates
  qpos0: np.ndarrau                """qpos values at default pose  (nq x 1)"""
  qpos_spring: np.ndarray          """reference pose for springs   (nq x 1)"""

  # bodies
  body_parentid: np.ndarray        """id of body's parent                    int*, (nbody x 1)"""
  body_rootid: np.ndarray          """id of root above body                  int*, (nbody x 1)"""
  body_weldid: np.ndarray          """id of body that this body is welded to int*, (nbody x 1)"""
  body_mocapid: np.ndarray         """id of mocap data; -1: none             int*, (nbody x 1)"""
  body_jntnum: np.ndarray          """number of joints for this body         int*, (nbody x 1)"""
  body_jntadr: np.ndarray          """start addr of joints; -1: no joints    int*, (nbody x 1)"""
  body_dofnum: np.ndarray          """number of motion degrees of freedom    int*, (nbody x 1)"""
  body_dofadr: np.ndarray          """start addr of dofs; -1: no dofs        int*, (nbody x 1)"""
  body_treeid: np.ndarray          """id of body's kinematic tree; -1: static  int* (nbody x 1)"""
  body_geomnum: np.ndarray         """number of geoms                          int* (nbody x 1)"""
  body_geomadr: np.ndarray         """start addr of geoms; -1: no geoms        int* (nbody x 1)"""

  body_simple: np.ndarray          """1: diag M; 2: diag M, sliders only       bool* (nbody x 1)"""
  body_sameframe: np.ndarray       """same frame as inertia (mjtSameframe)     bool* (nbody x 1)"""
  body_pos: np.ndarray             """position offset rel. to parent body      float* (nbody x 3)"""
  body_quat: np.ndarray            """orientation offset rel. to parent body   float* (nbody x 4)"""
  body_ipos: np.ndarray            """local position of center of mass         float* (nbody x 3)"""
  body_iquat: np.ndarray           """local orientation of inertia ellipsoid   float* (nbody x 4)"""
  body_mass: np.ndarray            """mass                                     float* (nbody x 1)"""
  body_subtreemass: np.ndarray     """mass of subtree starting at this body    float* (nbody x 1)"""
  body_inertia: np.ndarray         """diagonal inertia in ipos/iquat frame     float* (nbody x 3)"""
  body_invweight0: np.ndarray      """mean inv inert in qpos0 (trn, rot)       float* (nbody x 2)"""
  body_gravcomp: np.ndarray        """antigravity force, units of body weight  float* (nbody x 1)"""
  body_margin: np.ndarray          """MAX over all geom margins                float* (nbody x 1)"""
  body_user: np.ndarray            """user data                                float* (nbody x nuser_body)"""
  body_plugin: np.ndarray          """plugin instance id; -1: not in use       int* (nbody x 1)"""
  body_contype: np.ndarray         """OR over all geom contypes                int* (nbody x 1)"""
  body_conaffinity: np.ndarray     """OR over all geom conaffinities           int* (nbody x 1)"""
  body_bvhadr: np.ndarray          """address of bvh root                      int* (nbody x 1)"""
  body_bvhnum: np.ndarray          """number of bounding volumes               int* (nbody x 1)"""

  # bounding volume hierarchy
  bvh_depth: np.ndarray            """depth in the bounding volume hierarchy   int* (nbvh x 1)"""
  bvh_child: np.ndarray            """left and right children in tree          int* (nbvh x 2)"""
  bvh_nodeid: np.ndarray           """geom or elem id of node; -1: non-leaf    int* (nbvh x 1)"""
  bvh_aabb: np.ndarray             """local bounding box (center, size)        float* (nbvhstatic x 6)"""
  # joints
  jnt_type: np.ndarray             """type of joint (mjtJoint)                 int* (njnt x 1)"""
  jnt_qposadr: np.ndarray          """start addr in 'qpos' for joint's data    int* (njnt x 1)"""
  jnt_dofadr: np.ndarray           """start addr in 'qvel' for joint's data    int* (njnt x 1)"""
  jnt_bodyid: np.ndarray           """id of joint's body                       int* (njnt x 1)"""
  jnt_group: np.ndarray            """group for visibility                     int* (njnt x 1)"""
  jnt_limited: np.ndarray          """does joint have limits                   bool* (njnt x 1)"""
  jnt_actfrclimited: np.ndarray    """does joint have actuator force limits    bool* (njnt x 1)"""
  jnt_actgravcomp: np.ndarray      """is gravcomp force applied via actuators  bool* (njnt x 1)"""
  jnt_solref: np.ndarray           """constraint solver reference: limit       float* (njnt x mjNREF)"""
  jnt_solimp: np.ndarray           """constraint solver impedance: limit       float* (njnt x mjNIMP)"""
  jnt_pos: np.ndarray              """local anchor position                    float* (njnt x 3)"""
  jnt_axis: np.ndarray             """local joint axis                         float* (njnt x 3)"""
  jnt_stiffness: np.ndarray        """stiffness coefficient                    float* (njnt x 1)"""
  jnt_range: np.ndarray            """joint limits                             float* (njnt x 2)"""
  jnt_actfrcrange: np.ndarray      """range of total actuator force            float* (njnt x 2)"""
  jnt_margin: np.ndarray           """min distance for limit detection         float* (njnt x 1)"""
  jnt_user: np.ndarray             """user data                                float* (njnt x nuser_jnt)"""

  # dofs
  dof_bodyid: np.ndarray           """id of dof's body                         int* (nv x 1)"""
  dof_jntid: np.ndarray            """id of dof's joint                        int* (nv x 1)"""
  dof_parentid: np.ndarray         """id of dof's parent; -1: none             int* (nv x 1)"""
  dof_treeid: np.ndarray           """id of dof's kinematic tree               int* (nv x 1)"""
  dof_Madr: np.ndarray             """dof address in M-diagonal                int* (nv x 1)"""
  dof_simplenum: np.ndarray        """number of consecutive simple dofs        int* (nv x 1)"""
  dof_solref: np.ndarray           """constraint solver reference:frictionloss float* (nv x mjNREF)"""
  dof_solimp: np.ndarray           """constraint solver impedance:frictionloss float* (nv x mjNIMP)"""
  dof_frictionloss: np.ndarray     """dof friction loss                        float* (nv x 1)"""
  dof_armature: np.ndarray         """dof armature inertia/mass                float* (nv x 1)"""
  dof_damping: np.ndarray          """damping coefficient                      float* (nv x 1)"""
  dof_invweight0: np.ndarray       """diag. inverse inertia in qpos0           float* (nv x 1)"""
  dof_M0: np.ndarray               """diag. inertia in qpos0                   float* (nv x 1)"""

  # geoms
  geom_type: np.ndarray            """geometric type (mjtGeom)                 int* (ngeom x 1)"""
  geom_contype: np.ndarray         """geom contact type                        int* (ngeom x 1)"""
  geom_conaffinity: np.ndarray     """geom contact affinity                    int* (ngeom x 1)"""
  geom_condim: np.ndarray          """contact dimensionality (1, 3, 4, 6)      int* (ngeom x 1)"""
  geom_bodyid: np.ndarray          """id of geom's body                        int* (ngeom x 1)"""
  geom_dataid: np.ndarray          """id of geom's mesh/hfield; -1: none       int* (ngeom x 1)"""
  geom_matid: np.ndarray           """material id for rendering; -1: none      int* (ngeom x 1)"""
  geom_group: np.ndarray           """group for visibility                     int* (ngeom x 1)"""
  geom_priority: np.ndarray        """geom contact priority                    int* (ngeom x 1)"""
  geom_plugin: np.ndarray          """plugin instance id; -1: not in use       int* (ngeom x 1)"""
  geom_sameframe: np.ndarray       """same frame as body (mjtSameframe)        bool* (ngeom x 1)"""
  geom_solmix: np.ndarray          """mixing coef for solref/imp in geom pair  float* (ngeom x 1)"""
  geom_solref: np.ndarray          """constraint solver reference: contact     float* (ngeom x mjNREF)"""
  geom_solimp: np.ndarray          """constraint solver impedance: contact     float* (ngeom x mjNIMP)"""
  geom_size: np.ndarray            """geom-specific size parameters            float* (ngeom x 3)"""
  geom_aabb: np.ndarray            """bounding box, (center, size)             float* (ngeom x 6)"""
  geom_rbound: np.ndarray          """radius of bounding sphere                float* (ngeom x 1)"""
  geom_pos: np.ndarray             """local position offset rel. to body       float* (ngeom x 3)"""
  geom_quat: np.ndarray            """local orientation offset rel. to body    float* (ngeom x 4)"""
  geom_friction: np.ndarray        """friction for (slide, spin, roll)         float* (ngeom x 3)"""
  geom_margin: np.ndarray          """detect contact if dist<margin            float* (ngeom x 1)"""
  geom_gap: np.ndarray             """include in solver if dist<margin-gap     float* (ngeom x 1)"""
  geom_fluid: np.ndarray           """fluid interaction parameters             float* (ngeom x mjNFLUID)"""
  geom_user: np.ndarray            """user data                                float* (ngeom x nuser_geom)"""
  geom_rgba: np.ndarray            """rgba when material is omitted            float* (ngeom x 4)"""

  # sites
  site_type: np.ndarray            """geom type for rendering (mjtGeom)        int* (nsite x 1)"""
  site_bodyid: np.ndarray          """id of site's body                        int* (nsite x 1)"""
  site_matid: np.ndarray           """material id for rendering; -1: none      int* (nsite x 1)"""
  site_group: np.ndarray           """group for visibility                     int* (nsite x 1)"""
  site_sameframe: np.ndarray       """same frame as body (mjtSameframe)        bool* (nsite x 1)"""
  site_size: np.ndarray            """geom size for rendering                  float* (nsite x 3)"""
  site_pos: np.ndarray             """local position offset rel. to body       float* (nsite x 3)"""
  site_quat: np.ndarray            """local orientation offset rel. to body    float* (nsite x 4)"""
  site_user: np.ndarray            """user data                                float* (nsite x nuser_site)"""
  site_rgba: np.ndarray            """rgba when material is omitted            float* (nsite x 4)"""

  # cameras
  cam_mode: np.ndarray             """camera tracking mode (mjtCamLight)       int* (ncam x 1)"""
  cam_bodyid: np.ndarray           """id of camera's body                      int* (ncam x 1)"""
  cam_targetbodyid: np.ndarray     """id of targeted body; -1: none            int* (ncam x 1)"""
  cam_pos: np.ndarray              """position rel. to body frame              float* (ncam x 3)"""
  cam_quat: np.ndarray             """orientation rel. to body frame           float* (ncam x 4)"""
  cam_poscom0: np.ndarray          """global position rel. to sub-com in qpos0 float* (ncam x 3)"""
  cam_pos0: np.ndarray             """global position rel. to body in qpos0    float* (ncam x 3)"""
  cam_mat0: np.ndarray             """global orientation in qpos0              float* (ncam x 9)"""
  cam_orthographic: np.ndarray     """orthographic camera; 0: no, 1: yes       int* (ncam x 1)"""
  cam_fovy: np.ndarray             """y field-of-view (ortho ? len : deg)      float* (ncam x 1)"""
  cam_ipd: np.ndarray              """inter-pupilary distance                  float* (ncam x 1)"""
  cam_resolution: np.ndarray       """resolution: pixels [width, height]       int* (ncam x 2)"""
  cam_sensorsize: np.ndarray       """sensor size: length [width, height]      float* (ncam x 2)"""
  cam_intrinsic: np.ndarray        """[focal length; principal point]          float* (ncam x 4)"""
  cam_user: np.ndarray             """user data                                float* (ncam x nuser_cam)"""

  # lights
  light_mode: np.ndarray           """light tracking mode (mjtCamLight)        int* (nlight x 1)"""
  light_bodyid: np.ndarray         """id of light's body                       int* (nlight x 1)"""
  light_targetbodyid: np.ndarray   """id of targeted body; -1: none            int* (nlight x 1)"""
  light_directional: np.ndarray    """directional light                        bool* (nlight x 1)"""
  light_castshadow: np.ndarray     """does light cast shadows                  bool* (nlight x 1)"""
  light_bulbradius: np.ndarray     """light radius for soft shadows            float* (nlight x 1)"""
  light_active: np.ndarray         """is light on                              bool* (nlight x 1)"""
  light_pos: np.ndarray            """position rel. to body frame              float* (nlight x 3)"""
  light_dir: np.ndarray            """direction rel. to body frame             float* (nlight x 3)"""
  light_poscom0: np.ndarray        """global position rel. to sub-com in qpos0 float* (nlight x 3)"""
  light_pos0: np.ndarray           """global position rel. to body in qpos0    float* (nlight x 3)"""
  light_dir0: np.ndarray           """global direction in qpos0                float* (nlight x 3)"""
  light_attenuation: np.ndarray    """OpenGL attenuation (quadratic model)     float* (nlight x 3)"""
  light_cutoff: np.ndarray         """OpenGL cutoff                            float* (nlight x 1)"""
  light_exponent: np.ndarray       """OpenGL exponent                          float* (nlight x 1)"""
  light_ambient: np.ndarray        """ambient rgb (alpha=1)                    float* (nlight x 3)"""
  light_diffuse: np.ndarray        """diffuse rgb (alpha=1)                    float* (nlight x 3)"""
  light_specular: np.ndarray       """specular rgb (alpha=1)                   float* (nlight x 3)"""

  # flexes: contact properties
  flex_contype: np.ndarray         """flex contact type                        int* (nflex x 1)"""
  flex_conaffinity: np.ndarray     """flex contact affinity                    int* (nflex x 1)"""
  flex_condim: np.ndarray          """contact dimensionality (1, 3, 4, 6)      int* (nflex x 1)"""
  flex_priority: np.ndarray        """flex contact priority                    int* (nflex x 1)"""
  flex_solmix: np.ndarray          """mix coef for solref/imp in contact pair  float* (nflex x 1)"""
  flex_solref: np.ndarray          """constraint solver reference: contact     float* (nflex x mjNREF)"""
  flex_solimp: np.ndarray          """constraint solver impedance: contact     float* (nflex x mjNIMP)"""
  flex_friction: np.ndarray        """friction for (slide, spin, roll)         float* (nflex x 3)"""
  flex_margin: np.ndarray          """detect contact if dist<margin            float* (nflex x 1)"""
  flex_gap: np.ndarray             """include in solver if dist<margin-gap     float* (nflex x 1)"""
  flex_internal: np.ndarray        """internal flex collision enabled          bool* (nflex x 1)"""
  flex_selfcollide: np.ndarray     """self collision mode (mjtFlexSelf)        int* (nflex x 1)"""
  flex_activelayers: np.ndarray    """number of active element layers, 3D only int* (nflex x 1)"""

  # flexes: other properties
  flex_dim: np.ndarray             """1: lines, 2: triangles, 3: tetrahedra    int* (nflex x 1)"""
  flex_matid: np.ndarray           """material id for rendering                int* (nflex x 1)"""
  flex_group: np.ndarray           """group for visibility                     int* (nflex x 1)"""
  flex_vertadr: np.ndarray         """first vertex address                     int* (nflex x 1)"""
  flex_vertnum: np.ndarray         """number of vertices                       int* (nflex x 1)"""
  flex_edgeadr: np.ndarray         """first edge address                       int* (nflex x 1)"""
  flex_edgenum: np.ndarray         """number of edges                          int* (nflex x 1)"""
  flex_elemadr: np.ndarray         """first element address                    int* (nflex x 1)"""
  flex_elemnum: np.ndarray         """number of elements                       int* (nflex x 1)"""
  flex_elemdataadr: np.ndarray     """first element vertex id address          int* (nflex x 1)"""
  flex_elemedgeadr: np.ndarray     """first element edge id address            int* (nflex x 1)"""
  flex_shellnum: np.ndarray        """number of shells                         int* (nflex x 1)"""
  flex_shelldataadr: np.ndarray    """first shell data address                 int* (nflex x 1)"""
  flex_evpairadr: np.ndarray       """first evpair address                     int* (nflex x 1)"""
  flex_evpairnum: np.ndarray       """number of evpairs                        int* (nflex x 1)"""
  flex_texcoordadr: np.ndarray     """address in flex_texcoord; -1: none       int* (nflex x 1)"""
  flex_vertbodyid: np.ndarray      """vertex body ids                          int* (nflexvert x 1)"""
  flex_edge: np.ndarray            """edge vertex ids (2 per edge)             int* (nflexedge x 2)"""
  flex_elem: np.ndarray            """element vertex ids (dim+1 per elem)      int* (nflexelemdata x 1)"""
  flex_elemedge: np.ndarray        """element edge ids                         int* (nflexelemedge x 1)"""
  flex_elemlayer: np.ndarray       """element distance from surface, 3D only   int* (nflexelem x 1)"""
  flex_shell: np.ndarray           """shell fragment vertex ids (dim per frag) int* (nflexshelldata x 1)"""
  flex_evpair: np.ndarray          """(element, vertex) collision pairs        int* (nflexevpair x 2)"""
  flex_vert: np.ndarray            """vertex positions in local body frames    float* (nflexvert x 3)"""
  flex_vert0: np.ndarray           """vertex positions in qpos0 on [0, 1]^d    float* (nflexvert x 3)"""
  flexedge_length0: np.ndarray     """edge lengths in qpos0                    float* (nflexedge x 1)"""
  flexedge_invweight0: np.ndarray  """edge inv. weight in qpos0                float* (nflexedge x 1)"""
  flex_radius: np.ndarray          """radius around primitive element          float* (nflex x 1)"""
  flex_stiffness: np.ndarray       """finite element stiffness matrix          float* (nflexelem x 21)"""
  flex_damping: np.ndarray         """Rayleigh's damping coefficient           float* (nflex x 1)"""
  flex_edgestiffness: np.ndarray   """edge stiffness                           float* (nflex x 1)"""
  flex_edgedamping: np.ndarray     """edge damping                             float* (nflex x 1)"""
  flex_edgeequality: np.ndarray    """is edge equality constraint defined      bool* (nflex x 1)"""
  flex_rigid: np.ndarray           """are all verices in the same body         bool* (nflex x 1)"""
  flexedge_rigid: np.ndarray       """are both edge vertices in same body      bool* (nflexedge x 1)"""
  flex_centered: np.ndarray        """are all vertex coordinates (0,0,0)       bool* (nflex x 1)"""
  flex_flatskin: np.ndarray        """render flex skin with flat shading       bool* (nflex x 1)"""
  flex_bvhadr: np.ndarray          """address of bvh root; -1: no bvh          int* (nflex x 1)"""
  flex_bvhnum: np.ndarray          """number of bounding volumes               int* (nflex x 1)"""
  flex_rgba: np.ndarray            """rgba when material is omitted            float* (nflex x 4)"""
  flex_texcoord: np.ndarray        """vertex texture coordinates               float* (nflextexcoord x 2)"""

  # meshes
  mesh_vertadr: np.ndarray         """first vertex address                     int* (nmesh x 1)"""
  mesh_vertnum: np.ndarray         """number of vertices                       int* (nmesh x 1)"""
  mesh_faceadr: np.ndarray         """first face address                       int* (nmesh x 1)"""
  mesh_facenum: np.ndarray         """number of faces                          int* (nmesh x 1)"""
  mesh_bvhadr: np.ndarray          """address of bvh root                      int* (nmesh x 1)"""
  mesh_bvhnum: np.ndarray          """number of bvh                            int* (nmesh x 1)"""
  mesh_normaladr: np.ndarray       """first normal address                     int* (nmesh x 1)"""
  mesh_normalnum: np.ndarray       """number of normals                        int* (nmesh x 1)"""
  mesh_texcoordadr: np.ndarray     """texcoord data address; -1: no texcoord   int* (nmesh x 1)"""
  mesh_texcoordnum: np.ndarray     """number of texcoord                       int* (nmesh x 1)"""
  mesh_graphadr: np.ndarray        """graph data address; -1: no graph         int* (nmesh x 1)"""
  mesh_vert: np.ndarray            """vertex positions for all meshes          float* (nmeshvert x 3)"""
  mesh_normal: np.ndarray          """normals for all meshes                   float* (nmeshnormal x 3)"""
  mesh_texcoord: np.ndarray        """vertex texcoords for all meshes          float* (nmeshtexcoord x 2)"""
  mesh_face: np.ndarray            """vertex face data                         int* (nmeshface x 3)"""
  mesh_facenormal: np.ndarray      """normal face data                         int* (nmeshface x 3)"""
  mesh_facetexcoord: np.ndarray    """texture face data                        int* (nmeshface x 3)"""
  mesh_graph: np.ndarray           """convex graph data                        int* (nmeshgraph x 1)"""
  mesh_scale: np.ndarray           """scaling applied to asset vertices        float* (nmesh x 3)"""
  mesh_pos: np.ndarray             """translation applied to asset vertices    float* (nmesh x 3)"""
  mesh_quat: np.ndarray            """rotation applied to asset vertices       float* (nmesh x 4)"""
  mesh_pathadr: np.ndarray         """address of asset path for mesh; -1: none int* (nmesh x 1)"""

  # skins
  skin_matid: np.ndarray           """skin material id; -1: none               int* (nskin x 1)"""
  skin_group: np.ndarray           """group for visibility                     int* (nskin x 1)"""
  skin_rgba: np.ndarray            """skin rgba                                float* (nskin x 4)"""
  skin_inflate: np.ndarray         """inflate skin in normal direction         float* (nskin x 1)"""
  skin_vertadr: np.ndarray         """first vertex address                     int* (nskin x 1)"""
  skin_vertnum: np.ndarray         """number of vertices                       int* (nskin x 1)"""
  skin_texcoordadr: np.ndarray     """texcoord data address; -1: no texcoord   int* (nskin x 1)"""
  skin_faceadr: np.ndarray         """first face address                       int* (nskin x 1)"""
  skin_facenum: np.ndarray         """number of faces                          int* (nskin x 1)"""
  skin_boneadr: np.ndarray         """first bone in skin                       int* (nskin x 1)"""
  skin_bonenum: np.ndarray         """number of bones in skin                  int* (nskin x 1)"""
  skin_vert: np.ndarray            """vertex positions for all skin meshes     float* (nskinvert x 3)"""
  skin_texcoord: np.ndarray        """vertex texcoords for all skin meshes     float* (nskintexvert x 2)"""
  skin_face: np.ndarray            """triangle faces for all skin meshes       int* (nskinface x 3)"""
  skin_bonevertadr: np.ndarray     """first vertex in each bone                int* (nskinbone x 1)"""
  skin_bonevertnum: np.ndarray     """number of vertices in each bone          int* (nskinbone x 1)"""
  skin_bonebindpos: np.ndarray     """bind pos of each bone                    float* (nskinbone x 3)"""
  skin_bonebindquat: np.ndarray    """bind quat of each bone                   float* (nskinbone x 4)"""
  skin_bonebodyid: np.ndarray      """body id of each bone                     int* (nskinbone x 1)"""
  skin_bonevertid: np.ndarray      """mesh ids of vertices in each bone        int* (nskinbonevert x 1)"""
  skin_bonevertweight: np.ndarray  """weights of vertices in each bone         float* (nskinbonevert x 1)"""
  skin_pathadr: np.ndarray         """address of asset path for skin; -1: none int* (nskin x 1)"""

  # height fields
  hfield_size: np.ndarray          """(x, y, z_top, z_bottom)                  float* (nhfield x 4)"""
  hfield_nrow: np.ndarray          """number of rows in grid                   int* (nhfield x 1)"""
  hfield_ncol: np.ndarray          """number of columns in grid                int* (nhfield x 1)"""
  hfield_adr: np.ndarray           """address in hfield_data                   int* (nhfield x 1)"""
  hfield_data: np.ndarray          """elevation data                           float* (nhfielddata x 1)"""
  hfield_pathadr: np.ndarray       """address of hfield asset path; -1: none   int* (nhfield x 1)"""

  # textures
  tex_type: np.ndarray             """texture type (mjtTexture)                int* (ntex x 1)"""
  tex_height: np.ndarray           """number of rows in texture image          int* (ntex x 1)"""
  tex_width: np.ndarray            """number of columns in texture image       int* (ntex x 1)"""
  tex_nchannel: np.ndarray         """number of channels in texture image      int* (ntex x 1)"""
  tex_adr: np.ndarray              """start address in tex_data                int* (ntex x 1)"""
  tex_data: np.ndarray             """pixel values                             bool* (ntexdata x 1)"""
  tex_pathadr: np.ndarray          """address of texture asset path; -1: none  int* (ntex x 1)"""

  # materials
  mat_texid: np.ndarray            """indices of textures; -1: none            int* (nmat x mjNTEXROLE)"""
  mat_texuniform: np.ndarray       """make texture cube uniform                bool* (nmat x 1)"""
  mat_texrepeat: np.ndarray        """texture repetition for 2d mapping        float* (nmat x 2)"""
  mat_emission: np.ndarray         """emission (x rgb)                         float* (nmat x 1)"""
  mat_specular: np.ndarray         """specular (x white)                       float* (nmat x 1)"""
  mat_shininess: np.ndarray        """shininess coef                           float* (nmat x 1)"""
  mat_reflectance: np.ndarray      """reflectance (0: disable)                 float* (nmat x 1)"""
  mat_metallic: np.ndarray         """metallic coef                            float* (nmat x 1)"""
  mat_roughness: np.ndarray        """roughness coef                           float* (nmat x 1)"""
  mat_rgba: np.ndarray             """rgba                                     float* (nmat x 4)"""

  # predefined geom pairs for collision detection: np.ndarray has precedence over exclude
  pair_dim: np.ndarray             """contact dimensionality                   int* (npair x 1)"""
  pair_geom1: np.ndarray           """id of geom1                              int* (npair x 1)"""
  pair_geom2: np.ndarray           """id of geom2                              int* (npair x 1)"""
  pair_signature: np.ndarray       """body1 << 16 + body2                      int* (npair x 1)"""
  pair_solref: np.ndarray          """solver reference: contact normal         float* (npair x mjNREF)"""
  pair_solreffriction: np.ndarray  """solver reference: contact friction       float* (npair x mjNREF)"""
  pair_solimp: np.ndarray          """solver impedance: contact                float* (npair x mjNIMP)"""
  pair_margin: np.ndarray          """detect contact if dist<margin            float* (npair x 1)"""
  pair_gap: np.ndarray             """include in solver if dist<margin-gap     float* (npair x 1)"""
  pair_friction: np.ndarray        """tangent1, 2, spin, roll1, 2              float* (npair x 5)"""

  # excluded body pairs for collision detection
  exclude_signature: np.ndarray    """body1 << 16 + body2                      int* (nexclude x 1)"""

  # equality constraints
  eq_type: np.ndarray              """constraint type (mjtEq)                  int* (neq x 1)"""
  eq_obj1id: np.ndarray            """id of object 1                           int* (neq x 1)"""
  eq_obj2id: np.ndarray            """id of object 2                           int* (neq x 1)"""
  eq_objtype: np.ndarray           """type of both objects (mjtObj)            int* (neq x 1)"""
  eq_active0: np.ndarray           """initial enable/disable constraint state  bool* (neq x 1)"""
  eq_solref: np.ndarray            """constraint solver reference              float* (neq x mjNREF)"""
  eq_solimp: np.ndarray            """constraint solver impedance              float* (neq x mjNIMP)"""
  eq_data: np.ndarray              """numeric data for constraint              float* (neq x mjNEQDATA)"""

  # tendons
  tendon_adr: np.ndarray           """address of first object in tendon's path int* (ntendon x 1)"""
  tendon_num: np.ndarray           """number of objects in tendon's path       int* (ntendon x 1)"""
  tendon_matid: np.ndarray         """material id for rendering                int* (ntendon x 1)"""
  tendon_group: np.ndarray         """group for visibility                     int* (ntendon x 1)"""
  tendon_limited: np.ndarray       """does tendon have length limits           bool* (ntendon x 1)"""
  tendon_width: np.ndarray         """width for rendering                      float* (ntendon x 1)"""
  tendon_solref_lim: np.ndarray    """constraint solver reference: limit       float* (ntendon x mjNREF)"""
  tendon_solimp_lim: np.ndarray    """constraint solver impedance: limit       float* (ntendon x mjNIMP)"""
  tendon_solref_fri: np.ndarray    """constraint solver reference: friction    float* (ntendon x mjNREF)"""
  tendon_solimp_fri: np.ndarray    """constraint solver impedance: friction    float* (ntendon x mjNIMP)"""
  tendon_range: np.ndarray         """tendon length limits                     float* (ntendon x 2)"""
  tendon_margin: np.ndarray        """min distance for limit detection         float* (ntendon x 1)"""
  tendon_stiffness: np.ndarray     """stiffness coefficient                    float* (ntendon x 1)"""
  tendon_damping: np.ndarray       """damping coefficient                      float* (ntendon x 1)"""
  tendon_frictionloss: np.ndarray  """loss due to friction                     float* (ntendon x 1)"""
  tendon_lengthspring: np.ndarray  """spring resting length range              float* (ntendon x 2)"""
  tendon_length0: np.ndarray       """tendon length in qpos0                   float* (ntendon x 1)"""
  tendon_invweight0: np.ndarray    """inv. weight in qpos0                     float* (ntendon x 1)"""
  tendon_user: np.ndarray          """user data                                float* (ntendon x nuser_tendon)"""
  tendon_rgba: np.ndarray          """rgba when material is omitted            float* (ntendon x 4)"""

  # list of all wrap objects in tendon paths
  wrap_type: np.ndarray            """wrap object type (mjtWrap)               int* (nwrap x 1)"""
  wrap_objid: np.ndarray           """object id: geom, site, joint             int* (nwrap x 1)"""
  wrap_prm: np.ndarray             """divisor, joint coef, or site id          float* (nwrap x 1)"""

  # actuators
  actuator_trntype: np.ndarray     """transmission type (mjtTrn)               int* (nu x 1)"""
  actuator_dyntype: np.ndarray     """dynamics type (mjtDyn)                   int* (nu x 1)"""
  actuator_gaintype: np.ndarray    """gain type (mjtGain)                      int* (nu x 1)"""
  actuator_biastype: np.ndarray    """bias type (mjtBias)                      int* (nu x 1)"""
  actuator_trnid: np.ndarray       """transmission id: joint, tendon, site     int* (nu x 2)"""
  actuator_actadr: np.ndarray      """first activation address; -1: stateless  int* (nu x 1)"""
  actuator_actnum: np.ndarray      """number of activation variables           int* (nu x 1)"""
  actuator_group: np.ndarray       """group for visibility                     int* (nu x 1)"""
  actuator_ctrllimited: np.ndarray """is control limited                       bool* (nu x 1)"""
  actuator_forcelimited: np.ndarray"""is force limited                         bool* (nu x 1)"""
  actuator_actlimited: np.ndarray  """is activation limited                    bool* (nu x 1)"""
  actuator_dynprm: np.ndarray      """dynamics parameters                      float* (nu x mjNDYN)"""
  actuator_gainprm: np.ndarray     """gain parameters                          float* (nu x mjNGAIN)"""
  actuator_biasprm: np.ndarray     """bias parameters                          float* (nu x mjNBIAS)"""
  actuator_actearly: np.ndarray    """step activation before force             bool* (nu x 1)"""
  actuator_ctrlrange: np.ndarray   """range of controls                        float* (nu x 2)"""
  actuator_forcerange: np.ndarray  """range of forces                          float* (nu x 2)"""
  actuator_actrange: np.ndarray    """range of activations                     float* (nu x 2)"""
  actuator_gear: np.ndarray        """scale length and transmitted force       float* (nu x 6)"""
  actuator_cranklength: np.ndarray """crank length for slider-crank            float* (nu x 1)"""
  actuator_acc0: np.ndarray        """acceleration from unit force in qpos0    float* (nu x 1)"""
  actuator_length0: np.ndarray     """actuator length in qpos0                 float* (nu x 1)"""
  actuator_lengthrange: np.ndarray """feasible actuator length range           float* (nu x 2)"""
  actuator_user: np.ndarray        """user data                                float* (nu x nuser_actuator)"""
  actuator_plugin: np.ndarray      """plugin instance id; -1: not a plugin     int* (nu x 1)"""

  # sensors
  sensor_type: np.ndarray          """sensor type (mjtSensor)                  int* (nsensor x 1)"""
  sensor_datatype: np.ndarray      """numeric data type (mjtDataType)          int* (nsensor x 1)"""
  sensor_needstage: np.ndarray     """required compute stage (mjtStage)        int* (nsensor x 1)"""
  sensor_objtype: np.ndarray       """type of sensorized object (mjtObj)       int* (nsensor x 1)"""
  sensor_objid: np.ndarray         """id of sensorized object                  int* (nsensor x 1)"""
  sensor_reftype: np.ndarray       """type of reference frame (mjtObj)         int* (nsensor x 1)"""
  sensor_refid: np.ndarray         """id of reference frame; -1: global frame  int* (nsensor x 1)"""
  sensor_dim: np.ndarray           """number of scalar outputs                 int* (nsensor x 1)"""
  sensor_adr: np.ndarray           """address in sensor array                  int* (nsensor x 1)"""
  sensor_cutoff: np.ndarray        """cutoff for real and positive; 0: ignore  float* (nsensor x 1)"""
  sensor_noise: np.ndarray         """noise standard deviation                 float* (nsensor x 1)"""
  sensor_user: np.ndarray          """user data                                float* (nsensor x nuser_sensor)"""
  sensor_plugin: np.ndarray        """plugin instance id; -1: not a plugin     int* (nsensor x 1)"""

  # plugin instances
  plugin: np.ndarray               """globally registered plugin slot number   int* (nplugin x 1)"""
  plugin_stateadr: np.ndarray      """address in the plugin state array        int* (nplugin x 1)"""
  plugin_statenum: np.ndarray      """number of states in the plugin instance  int* (nplugin x 1)"""
  plugin_attr: bytes          """config attributes of plugin instances    char* (npluginattr x 1)"""
  plugin_attradr: np.ndarray       """address to each instance's config attrib int* (nplugin x 1)"""

  # custom numeric fields
  numeric_adr: np.ndarray          """address of field in numeric_data         int* (nnumeric x 1)"""
  numeric_size: np.ndarray         """size of numeric field                    int* (nnumeric x 1)"""
  numeric_data: np.ndarray         """array of all numeric fields              float* (nnumericdata x 1)"""

  # custom text fields
  text_adr: np.ndarray             """address of text in text_data             int* (ntext x 1)"""
  text_size: np.ndarray            """size of text field (strlen+1)            int* (ntext x 1)"""
  text_data: bytes            """array of all text fields (0-terminated)  char* (ntextdata x 1)"""

  # custom tuple fields
  tuple_adr: np.ndarray            """address of text in text_data             int* (ntuple x 1)"""
  tuple_size: np.ndarray           """number of objects in tuple               int* (ntuple x 1)"""
  tuple_objtype: np.ndarray        """array of object types in all tuples      int* (ntupledata x 1)"""
  tuple_objid: np.ndarray          """array of object ids in all tuples        int* (ntupledata x 1)"""
  tuple_objprm: np.ndarray         """array of object params in all tuples     float* (ntupledata x 1)"""

  # keyframes
  key_time: np.ndarray             """key time                                 float* (nkey x 1)"""
  key_qpos: np.ndarray             """key position                             float* (nkey x nq)"""
  key_qvel: np.ndarray             """key velocity                             float* (nkey x nv)"""
  key_act: np.ndarray              """key activation                           float* (nkey x na)"""
  key_mpos: np.ndarray             """key mocap position                       float* (nkey x nmocap*3)"""
  key_mquat: np.ndarray            """key mocap quaternion                     float* (nkey x nmocap*4)"""
  key_ctrl: np.ndarray             """key control                              float* (nkey x nu)"""

  # names
  name_bodyadr: np.ndarray         """body name pointers                       int* (nbody x 1)"""
  name_jntadr: np.ndarray          """joint name pointers                      int* (njnt x 1)"""
  name_geomadr: np.ndarray         """geom name pointers                       int* (ngeom x 1)"""
  name_siteadr: np.ndarray         """site name pointers                       int* (nsite x 1)"""
  name_camadr: np.ndarray          """camera name pointers                     int* (ncam x 1)"""
  name_lightadr: np.ndarray        """light name pointers                      int* (nlight x 1)"""
  name_flexadr: np.ndarray         """flex name pointers                       int* (nflex x 1)"""
  name_meshadr: np.ndarray         """mesh name pointers                       int* (nmesh x 1)"""
  name_skinadr: np.ndarray         """skin name pointers                       int* (nskin x 1)"""
  name_hfieldadr: np.ndarray       """hfield name pointers                     int* (nhfield x 1)"""
  name_texadr: np.ndarray          """texture name pointers                    int* (ntex x 1)"""
  name_matadr: np.ndarray          """material name pointers                   int* (nmat x 1)"""
  name_pairadr: np.ndarray         """geom pair name pointers                  int* (npair x 1)"""
  name_excludeadr: np.ndarray      """exclude name pointers                    int* (nexclude x 1)"""
  name_eqadr: np.ndarray           """equality constraint name pointers        int* (neq x 1)"""
  name_tendonadr: np.ndarray       """tendon name pointers                     int* (ntendon x 1)"""
  name_actuatoradr: np.ndarray     """actuator name pointers                   int* (nu x 1)"""
  name_sensoradr: np.ndarray       """sensor name pointers                     int* (nsensor x 1)"""
  name_numericadr: np.ndarray      """numeric name pointers                    int* (nnumeric x 1)"""
  name_textadr: np.ndarray         """text name pointers                       int* (ntext x 1)"""
  name_tupleadr: np.ndarray        """tuple name pointers                      int* (ntuple x 1)"""
  name_keyadr: np.ndarray          """keyframe name pointers                   int* (nkey x 1)"""
  name_pluginadr: np.ndarray       """plugin instance name pointers            int* (nplugin x 1)"""
  names: bytes                """names of all objects, 0-terminated       char* (nnames x 1)"""
  names_map: np.ndarray            """internal hash map of names               int* (nnames_map x 1)"""

  # paths
  paths: bytes                """paths to assets, 0-terminated            char* (npaths x 1)"""

class MjContact:
  """result of collision detection functions"""
  # contact parameters set by near-phase collision function
  dist: float                    """distance between nearest points; neg: penetration"""
  pos: np.ndarray                  """position of contact point: midpoint between geoms float* (3,)"""
  frame: np.ndarray                """normal is in [0-2], points from geom[0] to geom[1] float* (9,)"""

  # contact parameters set by mj_collideGeoms
  includemargin: float           """include if dist<includemargin=margin-gap"""
  friction: np.ndarray             """tangent1, 2, spin, roll1, 2 float* (5,)"""
  solref]: np.ndarray          """constraint solver reference, normal direction float* (mjNREF,)"""
  solreffriction: np.ndarray  """constraint solver reference, friction directions float* (mjNREF,)"""
  solimp: np.ndarray          """constraint solver impedance float* (mjNIMP,)"""

  # internal storage used by solver
  mu: float                      """friction of regularized cone, set by mj_makeConstraint"""
  H: np.ndarray                   """cone Hessian, set by mj_updateConstraint float* (36,)"""

  # contact descriptors set by mj_collideXXX
  dim: int                     """contact space dimensionality: 1, 3, 4 or 6"""
  geom1: int                   """id of geom 1; deprecated, use geom[0]"""
  geom2: int                   """id of geom 2; deprecated, use geom[1]"""
  geom: np.ndarray                 """geom ids; -1 for flex int* (2,)"""
  flex: np.ndarray                 """flex ids; -1 for geom"""
  elem: np.ndarray                 """element ids; -1 for geom or flex vertex int* (2,)"""
  vert: np.ndarray                 """vertex ids;  -1 for geom or flex element int* (2,)"""

  # flag set by mj_setContact or mj_instantiateContact
  exclude: int                 """0: include, 1: in gap, 2: fused, 3: no dofs"""

  # address computed by mj_instantiateContact"""
  efc_address: int             """address in efc; -1: not included"""
};
# typedef struct mjContact_ mjContact;



class MjData:
  def __init__(self, model: MjModel):
      ...

  # constant sizes
  narena: int            """size of the arena in bytes (inclusive of the stack)"""
  nbuffer: int           """size of main buffer in bytes"""
  nplugin: int           """number of plugin instances"""

  # stack pointer"""
  pstack: int            """first available mjtNum address in stack"""
  pbase: int             """value of pstack when mj_markStack was last called"""

  # arena pointer"""
  parena: int            """first available byte in arena"""

  # memory utilization stats"""
  maxuse_stack: int                      """maximum stack allocation in bytes"""
  maxuse_threadstack[mjMAXTHREAD]: int   """maximum stack allocation per thread in bytes"""
  maxuse_arena: int                      """maximum arena allocation in bytes"""
  maxuse_con: int                        """maximum number of contacts"""
  maxuse_efc: int                        """maximum number of scalar constraints"""

  # diagnostics"""
  warning: List[MjWarningStat]           """warning statistics"""
  timer: List[MjTimerStat]               """timer statistics"""

  # solver statistics
  solver: List[MjSolverStat]  """solver statistics per island, per iteration size=(mjNSOLVER * mjNISLAND,)"""
  solver_nisland: int           """number of islands processed by solver"""
  solver_niter: np.ndarray  """number of solver iterations, per island int* (mjNISLAND,)"""
  solver_nnz: np.ndarray    """number of non-zeros in Hessian or efc_AR, per island int* (mjNISLAND,)"""
  solver_fwdinv: np.ndarray         """forward-inverse comparison: qfrc, efc int* (2,)"""

  # variable sizes
  ne: int                """number of equality constraints"""
  nf: int                """number of friction constraints"""
  nl: int                """number of limit constraints"""
  nefc: int              """number of constraints"""
  nnzJ: int              """number of non-zeros in constraint Jacobian"""
  ncon: int              """number of detected contacts"""
  nisland: int           """number of detected constraint islands"""

  # global properties
  time: float              """simulation time"""
  energy: np.ndarray         """potential, kinetic energy float* (2,)"""

  # buffers
  buffer: Any            """main buffer; all pointers point in it                void* (nbuffer bytes)"""
  arena: List[Any]             """arena+stack buffer                     void* (nstack*sizeof(mjtNum) bytes)"""

  # state
  qpos: np.ndarray              """position                                         float* (nq x 1)"""
  qvel: np.ndarray              """velocity                                         float* (nv x 1)"""
  act: np.ndarray               """actuator activation                              float* (na x 1)"""
  qacc_warmstart: np.ndarray    """acceleration used for warmstart                  float* (nv x 1)"""
  plugin_state: np.ndarray      """plugin state                                     float* (npluginstate x 1)"""

  # control
  ctrl: np.ndarray              """control                                          float* (nu x 1)"""
  qfrc_applied: np.ndarray      """applied generalized force                        float* (nv x 1)"""
  xfrc_applied: np.ndarray      """applied Cartesian force/torque                   float* (nbody x 6)"""
  eq_active: np.ndarray        """enable/disable constraints                       bool* (neq x 1)"""

  # mocap data
  mocap_pos: np.ndarray         """positions of mocap bodies                        float* (nmocap x 3)"""
  mocap_quat: np.ndarray        """orientations of mocap bodies                     float* (nmocap x 4)"""

  # dynamics
  qacc: np.ndarray              """acceleration                                     float* (nv x 1)"""
  act_dot: np.ndarray           """time-derivative of actuator activation           float* (na x 1)"""

  # user data
  userdata: np.ndarray          """user data, not touched by engine                 float* (nuserdata x 1)"""

  # sensors
  sensordata: np.ndarray        """sensor data array                                float* (nsensordata x 1)"""

  # plugins
  plugin: np.ndarray         """copy of m->plugin, required for deletion         int* (nplugin x 1)"""
  plugin_data: List[Any]    """pointer to plugin-managed data structure         uintptr_t* (nplugin x 1)"""

  # computed by mj_fwdPosition/mj_kinematics
  xpos: np.ndarray              """cartesian position of body frame                 float* (nbody x 3)"""
  xquat: np.ndarray             """cartesian orientation of body frame              float* (nbody x 4)"""
  xmat: np.ndarray              """cartesian orientation of body frame              float* (nbody x 9)"""
  xipos: np.ndarray             """cartesian position of body com                   float* (nbody x 3)"""
  ximat: np.ndarray             """cartesian orientation of body inertia            float* (nbody x 9)"""
  xanchor: np.ndarray           """cartesian position of joint anchor               float* (njnt x 3)"""
  xaxis: np.ndarray             """cartesian joint axis                             float* (njnt x 3)"""
  geom_xpos: np.ndarray         """cartesian geom position                          float* (ngeom x 3)"""
  geom_xmat: np.ndarray         """cartesian geom orientation                       float* (ngeom x 9)"""
  site_xpos: np.ndarray         """cartesian site position                          float* (nsite x 3)"""
  site_xmat: np.ndarray         """cartesian site orientation                       float* (nsite x 9)"""
  cam_xpos: np.ndarray          """Cartesian camera position                        float* (ncam x 3)"""
  cam_xmat: np.ndarray          """Cartesian camera orientation                     float* (ncam x 9)"""
  light_xpos: np.ndarray        """Cartesian light position                         float* (nlight x 3)"""
  light_xdir: np.ndarray        """Cartesian light direction                        float* (nlight x 3)"""

  # computed by mj_fwdPosition/mj_comPos
  subtree_com: np.ndarray       """center of mass of each subtree                   float* (nbody x 3)"""
  cdof: np.ndarray              """com-based motion axis of each dof (rot:lin)      float* (nv x 6)"""
  cinert: np.ndarray            """com-based body inertia and mass                  float* (nbody x 10)"""

  # computed by mj_fwdPosition/mj_flex
  flexvert_xpos: np.ndarray     """Cartesian flex vertex positions                  float* (nflexvert x 3)"""
  flexelem_aabb: np.ndarray     """flex element bounding boxes (center, size)       float* (nflexelem x 6)"""
  flexedge_J_rownnz: np.ndarray """number of non-zeros in Jacobian row              int* (nflexedge x 1)"""
  flexedge_J_rowadr: np.ndarray """row start address in colind array                int* (nflexedge x 1)"""
  flexedge_J_colind: np.ndarray """column indices in sparse Jacobian                int* (nflexedge x nv)"""
  flexedge_J: np.ndarray        """flex edge Jacobian                               float* (nflexedge x nv)"""
  flexedge_length: np.ndarray   """flex edge lengths                                float* (nflexedge x 1)"""

  # computed by mj_fwdPosition/mj_tendon
  ten_wrapadr: np.ndarray       """start address of tendon's path                   int* (ntendon x 1)"""
  ten_wrapnum: np.ndarray       """number of wrap points in path                    int* (ntendon x 1)"""
  ten_J_rownnz: np.ndarray      """number of non-zeros in Jacobian row              int* (ntendon x 1)"""
  ten_J_rowadr: np.ndarray      """row start address in colind array                int* (ntendon x 1)"""
  ten_J_colind: np.ndarray      """column indices in sparse Jacobian                int* (ntendon x nv)"""
  ten_J: np.ndarray             """tendon Jacobian                                  float* (ntendon x nv)"""
  ten_length: np.ndarray        """tendon lengths                                   float* (ntendon x 1)"""
  wrap_obj: np.ndarray          """geom id; -1: site; -2: pulley                    int* (nwrap*2 x 1)"""
  wrap_xpos: np.ndarray         """Cartesian 3D points in all path                  float* (nwrap*2 x 3)"""

  # computed by mj_fwdPosition/mj_transmission
  actuator_length: np.ndarray   """actuator lengths                                 float* (nu x 1)"""
  actuator_moment: np.ndarray   """actuator moments                                 float* (nu x nv)"""

  # computed by mj_fwdPosition/mj_crb
  crb: np.ndarray               """com-based composite inertia and mass             float* (nbody x 10)"""
  qM: np.ndarray                """total inertia (sparse)                           float* (nM x 1)"""

  # computed by mj_fwdPosition/mj_factorM
  qLD: np.ndarray               """L'*D*L factorization of M (sparse)               float* (nM x 1)"""
  qLDiagInv: np.ndarray         """1/diag(D)                                        float* (nv x 1)"""
  qLDiagSqrtInv: np.ndarray     """1/sqrt(diag(D))                                  float* (nv x 1)"""

  # computed by mj_collisionTree
  bvh_aabb_dyn: np.ndarray     """global bounding box (center, size)               float* (nbvhdynamic x 6)"""
  bvh_active: np.ndarray       """volume has been added to collisions              bool* (nbvh x 1)"""

  # computed by mj_fwdVelocity
  flexedge_velocity: np.ndarray """flex edge velocities                             float* (nflexedge x 1)"""
  ten_velocity: np.ndarray      """tendon velocities                                float* (ntendon x 1)"""
  actuator_velocity: np.ndarray """actuator velocities                              float* (nu x 1)"""

  # computed by mj_fwdVelocity/mj_comVel
  cvel: np.ndarray              """com-based velocity (rot:lin)                     float* (nbody x 6)"""
  cdof_dot: np.ndarray          """time-derivative of cdof (rot:lin)                float* (nv x 6)"""

  # computed by mj_fwdVelocity/mj_rne (without acceleration)"""
  qfrc_bias: np.ndarray         """C(qpos,qvel)                                     float* (nv x 1)"""

  # computed by mj_fwdVelocity/mj_passive
  qfrc_spring: np.ndarray       """passive spring force                             float* (nv x 1)"""
  qfrc_damper: np.ndarray       """passive damper force                             float* (nv x 1)"""
  qfrc_gravcomp: np.ndarray     """passive gravity compensation force               float* (nv x 1)"""
  qfrc_fluid: np.ndarray        """passive fluid force                              float* (nv x 1)"""
  qfrc_passive: np.ndarray      """total passive force                              float* (nv x 1)"""

  # computed by mj_sensorVel/mj_subtreeVel if needed
  subtree_linvel: np.ndarray    """linear velocity of subtree com                   float* (nbody x 3)"""
  subtree_angmom: np.ndarray    """angular momentum about subtree com               float* (nbody x 3)"""

  # computed by mj_Euler or mj_implicit
  qH: np.ndarray                """L'*D*L factorization of modified M               float* (nM x 1)"""
  qHDiagInv: np.ndarray         """1/diag(D) of modified M                          float* (nv x 1)"""

  # computed by mj_resetData
  D_rownnz: np.ndarray          """non-zeros in each row                            int* (nv x 1)"""
  D_rowadr: np.ndarray          """address of each row in D_colind                  int* (nv x 1)"""
  D_colind: np.ndarray          """column indices of non-zeros                      int* (nD x 1)"""
  B_rownnz: np.ndarray          """non-zeros in each row                            int* (nbody x 1)"""
  B_rowadr: np.ndarray          """address of each row in B_colind                  int* (nbody x 1)"""
  B_colind: np.ndarray          """column indices of non-zeros                      int* (nB x 1)"""

  # computed by mj_implicit/mj_derivative
  qDeriv: np.ndarray            """d (passive + actuator - bias) / d qvel           float* (nD x 1)"""

  # computed by mj_implicit/mju_factorLUSparse
  qLU: np.ndarray               """sparse LU of (qM - dt*qDeriv)                    float* (nD x 1)"""

  # computed by mj_fwdActuation
  actuator_force: np.ndarray    """actuator force in actuation space                float* (nu x 1)"""
  qfrc_actuator: np.ndarray     """actuator force                                   float* (nv x 1)"""

  # computed by mj_fwdAcceleration
  qfrc_smooth: np.ndarray       """net unconstrained force                          float* (nv x 1)"""
  qacc_smooth: np.ndarray       """unconstrained acceleration                       float* (nv x 1)"""

  # computed by mj_fwdConstraint/mj_inverse
  qfrc_constraint: np.ndarray   """constraint force                                 float* (nv x 1)"""

  # computed by mj_inverse
  qfrc_inverse: np.ndarray      """net external force; should equal: qfrc_applied + J'*xfrc_applied + qfrc_actuator         float* (nv x 1)"""

  # computed by mj_sensorAcc/mj_rnePostConstraint if needed: np.ndarray rotation:translation format
  cacc: np.ndarray              """com-based acceleration                           float* (nbody x 6)"""
  cfrc_int: np.ndarray          """com-based interaction force with parent          float* (nbody x 6)"""
  cfrc_ext: np.ndarray          """com-based external force on body                 float* (nbody x 6)"""

  # computed by mj_collision
  contact: List[MjContact]        """list of all detected contacts                    (ncon x 1)"""

  # computed by mj_makeConstraint
  efc_type: np.ndarray          """constraint type (mjtConstraint)                  int* (nefc x 1)"""
  efc_id: np.ndarray            """id of object of specified type                   int* (nefc x 1)"""
  efc_J_rownnz: np.ndarray      """number of non-zeros in constraint Jacobian row   int* (nefc x 1)"""
  efc_J_rowadr: np.ndarray      """row start address in colind array                int* (nefc x 1)"""
  efc_J_rowsuper: np.ndarray    """number of subsequent rows in supernode           int* (nefc x 1)"""
  efc_J_colind: np.ndarray      """column indices in constraint Jacobian            int* (nnzJ x 1)"""
  efc_JT_rownnz: np.ndarray     """number of non-zeros in constraint Jacobian row T int* (nv x 1)"""
  efc_JT_rowadr: np.ndarray     """row start address in colind array              T int* (nv x 1)"""
  efc_JT_rowsuper: np.ndarray   """number of subsequent rows in supernode         T int* (nv x 1)"""
  efc_JT_colind: np.ndarray     """column indices in constraint Jacobian          T int* (nnzJ x 1)"""
  efc_J: np.ndarray             """constraint Jacobian                              float* (nnzJ x 1)"""
  efc_JT: np.ndarray            """constraint Jacobian transposed                   float* (nnzJ x 1)"""
  efc_pos: np.ndarray           """constraint position (equality, contact)          float* (nefc x 1)"""
  efc_margin: np.ndarray        """inclusion margin (contact)                       float* (nefc x 1)"""
  efc_frictionloss: np.ndarray  """frictionloss (friction)                          float* (nefc x 1)"""
  efc_diagApprox: np.ndarray    """approximation to diagonal of A                   float* (nefc x 1)"""
  efc_KBIP: np.ndarray          """stiffness, damping, impedance, imp'              float* (nefc x 4)"""
  efc_D: np.ndarray             """constraint mass                                  float* (nefc x 1)"""
  efc_R: np.ndarray             """inverse constraint mass                          float* (nefc x 1)"""
  tendon_efcadr: np.ndarray     """first efc address involving tendon; -1: none     int* (ntendon x 1)"""

  # computed by mj_island
  dof_island: np.ndarray        """island id of this dof; -1: none                  int* (nv x 1)"""
  island_dofnum: np.ndarray     """number of dofs in island                         int* (nisland x 1)"""
  island_dofadr: np.ndarray     """start address in island_dofind                   int* (nisland x 1)"""
  island_dofind: np.ndarray     """island dof indices; -1: none                     int* (nv x 1)"""
  dof_islandind: np.ndarray     """dof island indices; -1: none                     int* (nv x 1)"""
  efc_island: np.ndarray        """island id of this constraint                     int* (nefc x 1)"""
  island_efcnum: np.ndarray     """number of constraints in island                  int* (nisland x 1)"""
  island_efcadr: np.ndarray     """start address in island_efcind                   int* (nisland x 1)"""
  island_efcind: np.ndarray     """island constraint indices                        int* (nefc x 1)"""

  # computed by mj_projectConstraint (dual solver)"""
  efc_AR_rownnz: np.ndarray     """number of non-zeros in AR                        int* (nefc x 1)"""
  efc_AR_rowadr: np.ndarray     """row start address in colind array                int* (nefc x 1)"""
  efc_AR_colind: np.ndarray     """column indices in sparse AR                      int* (nefc x nefc)"""
  efc_AR: np.ndarray            """J*inv(M)*J' + R                                  float* (nefc x nefc)"""

  # computed by mj_fwdVelocity/mj_referenceConstraint
  efc_vel: np.ndarray           """velocity in constraint space: J*qvel             float* (nefc x 1)"""
  efc_aref: np.ndarray          """reference pseudo-acceleration                    float* (nefc x 1)"""

  # computed by mj_fwdConstraint/mj_inverse
  efc_b: np.ndarray            """linear cost term: J*qacc_smooth - aref            float* (nefc x 1)"""
  efc_force: np.ndarray        """constraint force in constraint space              float* (nefc x 1)"""
  efc_state: np.ndarray        """constraint state (mjtConstraintState)             int* (nefc x 1)"""

  # ThreadPool for multithreaded operations
  threadpool: Any  """thread pool handle"""


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
def mj_jac(m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, point: np.ndarray, body: int) -> None:
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

# mj_jacPointAxis
# void mj_jacPointAxis(const mjModel* m, mjData* d, mjtNum* jacPoint, mjtNum* jacAxis,
#                      const mjtNum point[3], const mjtNum axis[3], int body);
# Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.
def mj_jacPointAxis(m: MjModel, d: MjData, jacPoint: np.ndarray, jacAxis: np.ndarray, point: np.ndarray, axis: np.ndarray, body: int) -> None:
    """Compute translation end-effector Jacobian of point, and rotation Jacobian of axis."""
    ...

# mj_jacDot
# void mj_jacDot(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr,
#                const mjtNum point[3], int body);
# This function computes the time-derivative of an end-effector kinematic Jacobian computed by mj_jac. The minimal pipeline stages required for computation to be consistent with the current generalized positions and velocities mjData.{qpos, qvel} are mj_kinematics, mj_comPos, mj_comVel (in that order).
def mj_jacDot(m: MjModel, d: MjData, jacp: np.ndarray, jacr: np.ndarray, point: np.ndarray, body: int) -> None:
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
def mj_name2id(m: MjModel, type: int, name: str) -> int:
    """Get id of object with the specified mjtObj type and name, returns -1 if id not found."""
    ...

# mj_id2name
# const char* mj_id2name(const mjModel* m, int type, int id);
# Get name of object with the specified mjtObj type and id, returns NULL if name not found.
def mj_id2name(m: MjModel, type: int, id: int) -> str:
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
def mj_addM(m: MjModel, d: MjData, dst: np.ndarray, rownnz: np.ndarray, rowadr: np.ndarray, colind: np.ndarray) -> None:
    """Add inertia matrix to destination matrix. Destination can be sparse uncompressed, or dense when all int* are NULL"""
    ...

# mj_applyFT
# void mj_applyFT(const mjModel* m, mjData* d, const mjtNum force[3], const mjtNum torque[3],
#                 const mjtNum point[3], int body, mjtNum* qfrc_target);
# This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes we want to add the result to a different vector.
def mj_applyFT(m: MjModel, d: MjData, force: np.ndarray, torque: np.ndarray, point: np.ndarray, body: int, qfrc_target: np.ndarray) -> None:
    """This function can be used to apply a Cartesian force and torque to a point on a body, and add the result to the vector mjData.qfrc_applied of all applied forces. Note that the function requires a pointer to this vector, because sometimes we want to add the result to a different vector."""
    ...
 
# mj_objectVelocity
# void mj_objectVelocity(const mjModel* m, const mjData* d,
#                        int objtype, int objid, mjtNum res[6], int flg_local);
# Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
def mj_objectVelocity(m: MjModel, d: MjData, objtype: int, objid: int, res: np.ndarray, flg_local: int) -> None:
    """Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation."""
    ...

# mj_objectAcceleration
# void mj_objectAcceleration(const mjModel* m, const mjData* d,
#                            int objtype, int objid, mjtNum res[6], int flg_local);
# Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation. If acceleration or force sensors are not present in the model, mj_rnePostConstraint must be manually called in order to calculate mjData.cacc – the total body acceleration, including contributions from the constraint solver.
def mj_objectAcceleration(m: MjModel, d: MjData, objtype: int, objid: int, res: np.ndarray, flg_local: int) -> None:
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
def mj_geomDistance(m: MjModel, d: MjData, geom1: int, geom2: int, distmax: float, fromto: np.ndarray) -> float:
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
def mj_differentiatePos(m: MjModel, qvel: np.ndarray, dt: float, qpos1: np.ndarray, qpos2: np.ndarray) -> None:
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
def mj_local2Global(d: MjData, xpos: np.ndarray, xmat: np.ndarray, pos: np.ndarray, quat: np.ndarray, body: int, sameframe: int) -> None:
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
