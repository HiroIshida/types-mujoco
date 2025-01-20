import numpy as np
from typing import List, Any
# https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjmodel

class MjVisual: ...
class MjStatistic: ...

# struct mjOption_ {                // physics options
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

class MjData:
  # constant sizes
  narena: int            // size of the arena in bytes (inclusive of the stack)
  nbuffer: int           // size of main buffer in bytes
  nplugin: int           // number of plugin instances

  # stack pointer
  pstack: int            // first available mjtNum address in stack
  pbase: int             // value of pstack when mj_markStack was last called

  # arena pointer
  parena: int            // first available byte in arena

  # memory utilization stats
  maxuse_stack: int                      // maximum stack allocation in bytes
  maxuse_threadstack[mjMAXTHREAD]: int   // maximum stack allocation per thread in bytes
  maxuse_arena: int                      // maximum arena allocation in bytes
  maxuse_con: int                        // maximum number of contacts
  maxuse_efc: int                        // maximum number of scalar constraints

"""
  # diagnostics
  mjWarningStat warning[mjNWARNING];  // warning statistics
  mjTimerStat   timer[mjNTIMER];      // timer statistics

  # solver statistics
  mjSolverStat  solver[mjNISLAND*mjNSOLVER];  // solver statistics per island, per iteration
  int     solver_nisland;           // number of islands processed by solver
  int     solver_niter[mjNISLAND];  // number of solver iterations, per island
  int     solver_nnz[mjNISLAND];    // number of non-zeros in Hessian or efc_AR, per island
  mjtNum  solver_fwdinv[2];         // forward-inverse comparison: qfrc, efc

  # variable sizes
  int     ne;                // number of equality constraints
  int     nf;                // number of friction constraints
  int     nl;                // number of limit constraints
  int     nefc;              // number of constraints
  int     nnzJ;              // number of non-zeros in constraint Jacobian
  int     ncon;              // number of detected contacts
  int     nisland;           // number of detected constraint islands

  # global properties
  mjtNum  time;              // simulation time
  mjtNum  energy[2];         // potential, kinetic energy

  # buffers
  void*   buffer;            // main buffer; all pointers point in it                (nbuffer bytes)
  void*   arena;             // arena+stack buffer                     (nstack*sizeof(mjtNum) bytes)

  # state
  mjtNum* qpos;              // position                                         (nq x 1)
  mjtNum* qvel;              // velocity                                         (nv x 1)
  mjtNum* act;               // actuator activation                              (na x 1)
  mjtNum* qacc_warmstart;    // acceleration used for warmstart                  (nv x 1)
  mjtNum* plugin_state;      // plugin state                                     (npluginstate x 1)

  # control
  mjtNum* ctrl;              // control                                          (nu x 1)
  mjtNum* qfrc_applied;      // applied generalized force                        (nv x 1)
  mjtNum* xfrc_applied;      // applied Cartesian force/torque                   (nbody x 6)
  mjtByte* eq_active;        // enable/disable constraints                       (neq x 1)

  # mocap data
  mjtNum* mocap_pos;         // positions of mocap bodies                        (nmocap x 3)
  mjtNum* mocap_quat;        // orientations of mocap bodies                     (nmocap x 4)

  # dynamics
  mjtNum* qacc;              // acceleration                                     (nv x 1)
  mjtNum* act_dot;           // time-derivative of actuator activation           (na x 1)

  # user data
  mjtNum* userdata;          // user data, not touched by engine                 (nuserdata x 1)

  # sensors
  mjtNum* sensordata;        // sensor data array                                (nsensordata x 1)

  # plugins
  int*       plugin;         // copy of m->plugin, required for deletion         (nplugin x 1)
  uintptr_t* plugin_data;    // pointer to plugin-managed data structure         (nplugin x 1)

  # computed by mj_fwdPosition/mj_kinematics
  mjtNum* xpos;              // Cartesian position of body frame                 (nbody x 3)
  mjtNum* xquat;             // Cartesian orientation of body frame              (nbody x 4)
  mjtNum* xmat;              // Cartesian orientation of body frame              (nbody x 9)
  mjtNum* xipos;             // Cartesian position of body com                   (nbody x 3)
  mjtNum* ximat;             // Cartesian orientation of body inertia            (nbody x 9)
  mjtNum* xanchor;           // Cartesian position of joint anchor               (njnt x 3)
  mjtNum* xaxis;             // Cartesian joint axis                             (njnt x 3)
  mjtNum* geom_xpos;         // Cartesian geom position                          (ngeom x 3)
  mjtNum* geom_xmat;         // Cartesian geom orientation                       (ngeom x 9)
  mjtNum* site_xpos;         // Cartesian site position                          (nsite x 3)
  mjtNum* site_xmat;         // Cartesian site orientation                       (nsite x 9)
  mjtNum* cam_xpos;          // Cartesian camera position                        (ncam x 3)
  mjtNum* cam_xmat;          // Cartesian camera orientation                     (ncam x 9)
  mjtNum* light_xpos;        // Cartesian light position                         (nlight x 3)
  mjtNum* light_xdir;        // Cartesian light direction                        (nlight x 3)

  # computed by mj_fwdPosition/mj_comPos
  mjtNum* subtree_com;       // center of mass of each subtree                   (nbody x 3)
  mjtNum* cdof;              // com-based motion axis of each dof (rot:lin)      (nv x 6)
  mjtNum* cinert;            // com-based body inertia and mass                  (nbody x 10)

  # computed by mj_fwdPosition/mj_flex
  mjtNum* flexvert_xpos;     // Cartesian flex vertex positions                  (nflexvert x 3)
  mjtNum* flexelem_aabb;     // flex element bounding boxes (center, size)       (nflexelem x 6)
  int*    flexedge_J_rownnz; // number of non-zeros in Jacobian row              (nflexedge x 1)
  int*    flexedge_J_rowadr; // row start address in colind array                (nflexedge x 1)
  int*    flexedge_J_colind; // column indices in sparse Jacobian                (nflexedge x nv)
  mjtNum* flexedge_J;        // flex edge Jacobian                               (nflexedge x nv)
  mjtNum* flexedge_length;   // flex edge lengths                                (nflexedge x 1)

  # computed by mj_fwdPosition/mj_tendon
  int*    ten_wrapadr;       // start address of tendon's path                   (ntendon x 1)
  int*    ten_wrapnum;       // number of wrap points in path                    (ntendon x 1)
  int*    ten_J_rownnz;      // number of non-zeros in Jacobian row              (ntendon x 1)
  int*    ten_J_rowadr;      // row start address in colind array                (ntendon x 1)
  int*    ten_J_colind;      // column indices in sparse Jacobian                (ntendon x nv)
  mjtNum* ten_J;             // tendon Jacobian                                  (ntendon x nv)
  mjtNum* ten_length;        // tendon lengths                                   (ntendon x 1)
  int*    wrap_obj;          // geom id; -1: site; -2: pulley                    (nwrap*2 x 1)
  mjtNum* wrap_xpos;         // Cartesian 3D points in all path                  (nwrap*2 x 3)

  # computed by mj_fwdPosition/mj_transmission
  mjtNum* actuator_length;   // actuator lengths                                 (nu x 1)
  mjtNum* actuator_moment;   // actuator moments                                 (nu x nv)

   computed by mj_fwdPosition/mj_crb
  mjtNum* crb;               // com-based composite inertia and mass             (nbody x 10)
  mjtNum* qM;                // total inertia (sparse)                           (nM x 1)

  // computed by mj_fwdPosition/mj_factorM
  mjtNum* qLD;               // L'*D*L factorization of M (sparse)               (nM x 1)
  mjtNum* qLDiagInv;         // 1/diag(D)                                        (nv x 1)
  mjtNum* qLDiagSqrtInv;     // 1/sqrt(diag(D))                                  (nv x 1)

  // computed by mj_collisionTree
  mjtNum*  bvh_aabb_dyn;     // global bounding box (center, size)               (nbvhdynamic x 6)
  mjtByte* bvh_active;       // volume has been added to collisions              (nbvh x 1)

  //-------------------- POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity
  mjtNum* flexedge_velocity; // flex edge velocities                             (nflexedge x 1)
  mjtNum* ten_velocity;      // tendon velocities                                (ntendon x 1)
  mjtNum* actuator_velocity; // actuator velocities                              (nu x 1)

  // computed by mj_fwdVelocity/mj_comVel
  mjtNum* cvel;              // com-based velocity (rot:lin)                     (nbody x 6)
  mjtNum* cdof_dot;          // time-derivative of cdof (rot:lin)                (nv x 6)

  // computed by mj_fwdVelocity/mj_rne (without acceleration)
  mjtNum* qfrc_bias;         // C(qpos,qvel)                                     (nv x 1)

  // computed by mj_fwdVelocity/mj_passive
  mjtNum* qfrc_spring;       // passive spring force                             (nv x 1)
  mjtNum* qfrc_damper;       // passive damper force                             (nv x 1)
  mjtNum* qfrc_gravcomp;     // passive gravity compensation force               (nv x 1)
  mjtNum* qfrc_fluid;        // passive fluid force                              (nv x 1)
  mjtNum* qfrc_passive;      // total passive force                              (nv x 1)

  // computed by mj_sensorVel/mj_subtreeVel if needed
  mjtNum* subtree_linvel;    // linear velocity of subtree com                   (nbody x 3)
  mjtNum* subtree_angmom;    // angular momentum about subtree com               (nbody x 3)

  // computed by mj_Euler or mj_implicit
  mjtNum* qH;                // L'*D*L factorization of modified M               (nM x 1)
  mjtNum* qHDiagInv;         // 1/diag(D) of modified M                          (nv x 1)

  // computed by mj_resetData
  int*    D_rownnz;          // non-zeros in each row                            (nv x 1)
  int*    D_rowadr;          // address of each row in D_colind                  (nv x 1)
  int*    D_colind;          // column indices of non-zeros                      (nD x 1)
  int*    B_rownnz;          // non-zeros in each row                            (nbody x 1)
  int*    B_rowadr;          // address of each row in B_colind                  (nbody x 1)
  int*    B_colind;          // column indices of non-zeros                      (nB x 1)

  // computed by mj_implicit/mj_derivative
  mjtNum* qDeriv;            // d (passive + actuator - bias) / d qvel           (nD x 1)

  // computed by mj_implicit/mju_factorLUSparse
  mjtNum* qLU;               // sparse LU of (qM - dt*qDeriv)                    (nD x 1)

  //-------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdActuation
  mjtNum* actuator_force;    // actuator force in actuation space                (nu x 1)
  mjtNum* qfrc_actuator;     // actuator force                                   (nv x 1)

  // computed by mj_fwdAcceleration
  mjtNum* qfrc_smooth;       // net unconstrained force                          (nv x 1)
  mjtNum* qacc_smooth;       // unconstrained acceleration                       (nv x 1)

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum* qfrc_constraint;   // constraint force                                 (nv x 1)

  // computed by mj_inverse
  mjtNum* qfrc_inverse;      // net external force; should equal:                (nv x 1)
                             // qfrc_applied + J'*xfrc_applied + qfrc_actuator

  // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
  mjtNum* cacc;              // com-based acceleration                           (nbody x 6)
  mjtNum* cfrc_int;          // com-based interaction force with parent          (nbody x 6)
  mjtNum* cfrc_ext;          // com-based external force on body                 (nbody x 6)

  //-------------------- arena-allocated: POSITION dependent

  // computed by mj_collision
  mjContact* contact;        // list of all detected contacts                    (ncon x 1)

  // computed by mj_makeConstraint
  int*    efc_type;          // constraint type (mjtConstraint)                  (nefc x 1)
  int*    efc_id;            // id of object of specified type                   (nefc x 1)
  int*    efc_J_rownnz;      // number of non-zeros in constraint Jacobian row   (nefc x 1)
  int*    efc_J_rowadr;      // row start address in colind array                (nefc x 1)
  int*    efc_J_rowsuper;    // number of subsequent rows in supernode           (nefc x 1)
  int*    efc_J_colind;      // column indices in constraint Jacobian            (nnzJ x 1)
  int*    efc_JT_rownnz;     // number of non-zeros in constraint Jacobian row T (nv x 1)
  int*    efc_JT_rowadr;     // row start address in colind array              T (nv x 1)
  int*    efc_JT_rowsuper;   // number of subsequent rows in supernode         T (nv x 1)
  int*    efc_JT_colind;     // column indices in constraint Jacobian          T (nnzJ x 1)
  mjtNum* efc_J;             // constraint Jacobian                              (nnzJ x 1)
  mjtNum* efc_JT;            // constraint Jacobian transposed                   (nnzJ x 1)
  mjtNum* efc_pos;           // constraint position (equality, contact)          (nefc x 1)
  mjtNum* efc_margin;        // inclusion margin (contact)                       (nefc x 1)
  mjtNum* efc_frictionloss;  // frictionloss (friction)                          (nefc x 1)
  mjtNum* efc_diagApprox;    // approximation to diagonal of A                   (nefc x 1)
  mjtNum* efc_KBIP;          // stiffness, damping, impedance, imp'              (nefc x 4)
  mjtNum* efc_D;             // constraint mass                                  (nefc x 1)
  mjtNum* efc_R;             // inverse constraint mass                          (nefc x 1)
  int*    tendon_efcadr;     // first efc address involving tendon; -1: none     (ntendon x 1)

  // computed by mj_island
  int*    dof_island;        // island id of this dof; -1: none                  (nv x 1)
  int*    island_dofnum;     // number of dofs in island                         (nisland x 1)
  int*    island_dofadr;     // start address in island_dofind                   (nisland x 1)
  int*    island_dofind;     // island dof indices; -1: none                     (nv x 1)
  int*    dof_islandind;     // dof island indices; -1: none                     (nv x 1)
  int*    efc_island;        // island id of this constraint                     (nefc x 1)
  int*    island_efcnum;     // number of constraints in island                  (nisland x 1)
  int*    island_efcadr;     // start address in island_efcind                   (nisland x 1)
  int*    island_efcind;     // island constraint indices                        (nefc x 1)

  // computed by mj_projectConstraint (dual solver)
  int*    efc_AR_rownnz;     // number of non-zeros in AR                        (nefc x 1)
  int*    efc_AR_rowadr;     // row start address in colind array                (nefc x 1)
  int*    efc_AR_colind;     // column indices in sparse AR                      (nefc x nefc)
  mjtNum* efc_AR;            // J*inv(M)*J' + R                                  (nefc x nefc)

  //-------------------- arena-allocated: POSITION, VELOCITY dependent

  // computed by mj_fwdVelocity/mj_referenceConstraint
  mjtNum* efc_vel;           // velocity in constraint space: J*qvel             (nefc x 1)
  mjtNum* efc_aref;          // reference pseudo-acceleration                    (nefc x 1)

  //-------------------- arena-allocated: POSITION, VELOCITY, CONTROL/ACCELERATION dependent

  // computed by mj_fwdConstraint/mj_inverse
  mjtNum* efc_b;            // linear cost term: J*qacc_smooth - aref            (nefc x 1)
  mjtNum* efc_force;        // constraint force in constraint space              (nefc x 1)
  int*    efc_state;        // constraint state (mjtConstraintState)             (nefc x 1)

  // ThreadPool for multithreaded operations
  uintptr_t threadpool;
};
typedef struct mjData_ mjData;
"""
