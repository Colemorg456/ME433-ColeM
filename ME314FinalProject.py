import sympy as sym
import numpy as np
from matplotlib import pyplot as plt
##############################################################################################
# If you're using Google Colab, uncomment this section by selecting the whole section and press
# ctrl+'/' on your and keyboard. Run it before you start programming, this will enable the nice
# LaTeX "display()" function for you. If you're using the local Jupyter environment, leave it alone
##############################################################################################
import sympy as sym
def custom_latex_printer(exp,**options):
    from google.colab.output._publish import javascript
    url = "https://cdnjs.cloudflare.com/ajax/libs/mathjax/3.1.1/latest.js?config=TeX-AMS_HTML"
    javascript(url=url)
    return sym.printing.latex(exp,**options)
sym.init_printing(use_latex="mathjax",latex_printer=custom_latex_printer)

#Helper functions used later on

def integrate(f, xt, dt ,t):
    """
    This function takes in an initial condition x(t) and a timestep dt,
    as well as a dynamical system f(x) that outputs a vector of the
    same dimension as x(t). It outputs a vector x(t+dt) at the future
    time step.

    Parameters
    ============
    dyn: Python function
        derivate of the system at a given step x(t),
        it can considered as \dot{x}(t) = func(x(t))
    x0: NumPy array
        current step x(t)
    dt:
        step size for integration
    time:
        step time

    Return
    ============
    new_x:
        value of x(t+dt) integrated from x(t)
    """
    k1 = dt*f(xt,t)
    k2 = dt*f(xt+k1/2.0,t)
    k3 = dt*f(xt+k2/2.0,t)
    k4 = dt*f(xt+k3,t)
    new_xt = xt+(1/6.0)*(k1+2.0*k2+2.0*k3+k4)
    return new_xt

#unhat function and inverting SE3 matrix functions for the Lagrangian calculation
def unhat(vhat):
    return sym.Matrix([vhat[0,3],vhat[1,3],vhat[2,3],vhat[0,2],vhat[2,1],vhat[1,0]])

def SE3_inverse(M):
    R = M[:3,:3]
    p = M[:3,3]
    M_inv = sym.eye(4)
    M_inv[:3,:3] = R.T
    M_inv[:3,3] = -R.T*p
    return M_inv
## Your code goes here

#Hardcoding in these numbers to avoid sybmolic errors
g = 9.81
L_box = 3
L_jack = 0.8
m_box = 5
m_jack = 1
J_box = 10
J_jack = 0.5

#time variables
t = sym.symbols(r't')
x_b = sym.Function(r'x_box')(t)
x_b_dot = sym.Function(r'\dot{x}_box')(t)
x_b_ddot = sym.Function(r'\ddot{x}_box')(t)

y_b = sym.Function(r'y_box')(t)
y_b_dot = sym.Function(r'\dot{y}_box')(t)
y_b_ddot = sym.Function(r'\ddot{y}_box')(t)

th_b = sym.Function(r'\theta_box')(t)
th_b_dot = sym.Function(r'\dot{\theta}_box')(t)
th_b_ddot = sym.Function(r'\ddot{\theta}_box')(t)

x_j = sym.Function(r'x_jack')(t)
x_j_dot = sym.Function(r'\dot{x}_jack')(t)
x_j_ddot = sym.Function(r'\ddot{x}_jack')(t)

y_j = sym.Function(r'y_jack')(t)
y_j_dot = sym.Function(r'\dot{y}_jack')(t)
y_j_ddot = sym.Function(r'\ddot{y}_jack')(t)

th_j = sym.Function(r'\theta_jack')(t)
th_j_dot = sym.Function(r'\dot{\theta}_jack')(t)
th_j_ddot = sym.Function(r'\ddot{\theta}_jack')(t)

q = sym.Matrix([x_b,y_b,th_b,x_j,y_j,th_j])
qdot = q.diff(t)
qddot = qdot.diff(t)
#-----------------------------------------
#Rotation matrices
RWBox = sym.Matrix([[sym.cos(th_b),-sym.sin(th_b),0],[sym.sin(th_b),sym.cos(th_b),0],[0,0,1]])
RWJack = sym.Matrix([[sym.cos(th_j),-sym.sin(th_j),0],[sym.sin(th_j),sym.cos(th_j),0],[0,0,1]])
R_I = sym.eye(3)

#Center in world frame
PWBox = sym.Matrix([x_b,y_b,0])
PWJack = sym.Matrix([x_j,y_j,0])

#Translation Matrices, L/2 for the midpoints of the box and the jack
Pba = sym.Matrix([0,-L_box/2,0])
Pbb = sym.Matrix([L_box/2,0,0])
Pbc = sym.Matrix([0,L_box/2,0])
Pbd = sym.Matrix([-L_box/2,0,0])

Pje = sym.Matrix([0,-L_jack/2,0])
Pjf = sym.Matrix([L_jack/2,0,0])
Pjg = sym.Matrix([0,L_jack/2,0])
Pjh = sym.Matrix([-L_jack/2,0,0])

##----------------------------------------
#world frame to jack/box frame
gWBox = sym.Matrix([[RWBox,PWBox],[0,0,0,1]])
gWJack = sym.Matrix([[RWJack,PWJack],[0,0,0,1]])

#Euclidian groups
gBA = sym.Matrix([[R_I, Pba],[0,0,0,1]])
gBB = sym.Matrix([[R_I, Pbb],[0,0,0,1]])
gBC = sym.Matrix([[R_I, Pbc],[0,0,0,1]])
gBD = sym.Matrix([[R_I, Pbd],[0,0,0,1]])
gJE = sym.Matrix([[R_I, Pje],[0,0,0,1]])
gJF = sym.Matrix([[R_I, Pjf],[0,0,0,1]])
gJG = sym.Matrix([[R_I, Pjg],[0,0,0,1]])
gJH = sym.Matrix([[R_I, Pjh],[0,0,0,1]])

#transform for world frame to box/jack corner
gWA = gWBox*gBA
gWB = gWBox*gBB
gWC = gWBox*gBC
gWD = gWBox*gBD

gWE = gWJack*gJE
gWF = gWJack*gJF
gWG = gWJack*gJG
gWH = gWJack*gJH

##-------------------------------------------------
#calculating V^b for each point to find the total KE
VBoxA = unhat((SE3_inverse(gWA))*gWA.diff(t))
VBoxB = unhat((SE3_inverse(gWB))*gWB.diff(t))
VBoxC = unhat((SE3_inverse(gWC))*gWC.diff(t))
VBoxD = unhat((SE3_inverse(gWD))*gWD.diff(t))
VJackE = unhat((SE3_inverse(gWE))*gWE.diff(t))
VJackF = unhat((SE3_inverse(gWF))*gWF.diff(t))
VJackG = unhat((SE3_inverse(gWG))*gWG.diff(t))
VJackH = unhat((SE3_inverse(gWH))*gWH.diff(t))

#Inertial matrix
I_B = sym.diag(m_box,m_box, m_box,0,0,J_box)
I_J = sym.diag(m_jack,m_jack,m_jack,0,0,J_jack)

#Calculate KE using the long matrix equation version
KE_Box = (VBoxA.T*I_B *VBoxA)/2 + (VBoxB.T*I_B*VBoxB)/2 + (VBoxC.T*I_B*VBoxC)/2 + (VBoxD.T*I_B*VBoxD)/2
KE_Jack = (VJackE.T * I_J * VJackE)/2 + (VJackF.T * I_J * VJackF)/2 + (VJackG.T * I_J * VJackG)/2 + (VJackH.T * I_J * VJackH)/2

#Compute Lagrangian
KE = KE_Box+KE_Jack
PE = (4*m_box*g*y_b)+ (4*m_jack*g*y_j)
L = sym.Matrix([KE[0] - PE])

##-------------------------------------------------
#Calculate EL Eqs
dLdq = L.jacobian(q)
dLdqdot = L.jacobian(qdot)
ddt = dLdqdot.diff(t)
EL = ddt-dLdq
simple_EL = sym.simplify(EL)

#Make external force to go on the box to make it shake
Fx = 80*sym.cos(4*t) #making it oscillate using a cos wave
Fy = (4*m_box*g)+(4*m_jack*g) #offset the gravity so it all doesnt just fall
FT = 60 #Making it spin
Q = sym.Matrix([Fx,Fy,FT,0,0,0])

lhs = simple_EL.T
rhs = sym.simplify(Q)
EL_eq = sym.Eq(lhs, rhs)
EL_solution = sym.solve(EL_eq,qddot,dict=True)

##-------------------------------------------------
#subbing in dummy variables for everything
x_b_d, y_b_d, th_b_d, x_j_d, y_j_d, th_j_d = sym.symbols(r'x_b_d, y_b_d, th_b_d, x_j_d, y_j_d, th_j_d')
x_bdot_d, y_bdot_d, th_bdot_d, x_jdot_d, y_jdot_d, th_jdot_d = sym.symbols(r'x_bdot_d, y_bdot_d, th_bdot_d, x_jdot_d, y_jdot_d, th_jdot_d')
x_bddot_d, y_bddot_d, th_bddot_d, x_jddot_d, y_jddot_d, th_jddot_d = sym.symbols(r'x_bddot_d, y_bddot_d, th_bddot_d, x_jddot_d, y_jddot_d, th_jddot_d')

qdumb = sym.Matrix([x_b_d, y_b_d, th_b_d, x_j_d, y_j_d, th_j_d])
qdot_dummy = sym.Matrix([x_bdot_d, y_bdot_d, th_bdot_d, x_jdot_d, y_jdot_d, th_jdot_d])
qddot_dummy = sym.Matrix([x_bddot_d, y_bddot_d, th_bddot_d, x_jddot_d, y_jddot_d, th_jddot_d])

dummy_vars = {q[0]:x_b_d, q[1]:y_b_d, q[2]:th_b_d, q[3]:x_j_d, q[4]:y_j_d, q[5]:th_j_d,
           qdot[0]:x_bdot_d, qdot[1]:y_bdot_d, qdot[2]:th_bdot_d, qdot[3]:x_jdot_d, qdot[4]:y_jdot_d, qdot[5]:th_jdot_d,
           qddot[0]:x_bddot_d, qddot[1]:y_bddot_d, qddot[2]:th_bddot_d, qddot[3]:x_jddot_d, qddot[4]:y_jddot_d, qddot[5]:th_jddot_d}

EL_dummy = sym.simplify(EL_eq.subs(dummy_vars))
dummy_sol = sym.solve(EL_dummy, qddot_dummy, dict=True)

for i in dummy_sol:
  for j in qddot_dummy:
    expr = sym.simplify(dummy_sol[0][j])
    sym.Eq(j,expr)

dummy_t_vars = [x_b_d, y_b_d, th_b_d, x_j_d, y_j_d, th_j_d, x_bdot_d, y_bdot_d, th_bdot_d, x_jdot_d, y_jdot_d, th_jdot_d, t]
x_b_num = sym.lambdify(dummy_t_vars,dummy_sol[0][x_bddot_d])
y_b_num = sym.lambdify(dummy_t_vars, dummy_sol[0][y_bddot_d])
th_b_num = sym.lambdify(dummy_t_vars,dummy_sol[0][th_bddot_d])
x_j_num = sym.lambdify(dummy_t_vars,dummy_sol[0][x_jddot_d])
y_j_num = sym.lambdify(dummy_t_vars, dummy_sol[0][y_jddot_d])
th_j_num = sym.lambdify(dummy_t_vars,dummy_sol[0][th_jddot_d])
##-------------------------------------------------
#Making Impact equations
#make the dummy variables for impacts
x_bdot_p, y_bdot_p, th_bdot_p, x_jdot_p, y_jdot_p, th_jdot_p,λ = sym.symbols(r'x_bdot_p y_bdot_p th_bdot_p x_jdot_p y_jdot_p th_jdot_p λ')
x_bdot_m, y_bdot_m, th_bdot_m, x_jdot_m, y_jdot_m, th_jdot_m = sym.symbols(r'x_bdot_m y_bdot_m th_bdot_m x_jdot_m y_jdot_m th_jdot_m')
subs_plus = {x_bdot_d:x_bdot_p, y_bdot_d:y_bdot_p, th_bdot_d:th_bdot_p, x_jdot_d:x_jdot_p, y_jdot_d:y_jdot_p, th_jdot_d:th_jdot_p}
subs_minus = {x_bdot_d:x_bdot_m, y_bdot_d:y_bdot_m, th_bdot_d:th_bdot_m, x_jdot_d:x_jdot_m, y_jdot_d:y_jdot_m, th_jdot_d:th_jdot_m}

#origin point for the world frame
origin = sym.Matrix([0,0,0,1])
#transforms to move every point on the jack into the world frame
#after that, transforming the jack into the box frame to check for impacts
PointE = gWE*origin
PointF = gWF*origin
PointG = gWG*origin
PointH = gWH*origin
BF_E = SE3_inverse(gWBox)*PointE
BF_F = SE3_inverse(gWBox)*PointF
BF_G = SE3_inverse(gWBox)*PointG
BF_H = SE3_inverse(gWBox)*PointH

#manually having to hardcode in every constraint between jack and the box
#to check phi later on to see if theres an impact with a box wall or not
Phi_E_bot = BF_E[1]+L_box/2
Phi_E_right = -BF_E[0]+L_box/2
Phi_E_top = -BF_E[1]+L_box/2
Phi_E_left = BF_E[0]+L_box/2
Phi_F_bot = BF_F[1]+L_box/2
Phi_F_right = -BF_F[0]+L_box/2
Phi_F_top = -BF_F[1]+L_box/2
Phi_F_left = BF_F[0]+L_box/2
Phi_G_bot = BF_G[1]+L_box/2
Phi_G_right = -BF_G[0]+L_box/2
Phi_G_top = -BF_G[1]+L_box/2
Phi_G_left = BF_G[0]+L_box/2
Phi_H_bot = BF_H[1]+L_box/2
Phi_H_right = -BF_H[0]+L_box/2
Phi_H_top = -BF_H[1]+L_box/2
Phi_H_left = BF_H[0]+L_box/2

#putting every phi into a list so it can be ran through later to check for impacts
phi_list = sym.Matrix([Phi_E_bot, Phi_E_right, Phi_E_top, Phi_E_left, Phi_F_bot, Phi_F_right, Phi_F_top, Phi_F_left,
                   Phi_G_bot, Phi_G_right, Phi_G_top, Phi_G_left, Phi_H_bot, Phi_H_right, Phi_H_top, Phi_H_left])

#calculate p and H for impact equations calculations
p_dummy = sym.simplify((L.jacobian(qdot)).subs(dummy_vars))
p_plus = p_dummy.subs(subs_plus)
p_minus = p_dummy.subs(subs_minus)
p = p_plus-p_minus

H_dummy = sym.simplify(p_dummy*qdot_dummy - L.subs(dummy_vars))
H_plus = H_dummy.subs(subs_plus)
H_minus = H_dummy.subs(subs_minus)
H = H_plus-H_minus

#call function to calculate impact equations for whatever phi is being looked at
def impact_equations(Phi):
    dphidq = Phi.diff(q)
    dphidq_dummy = dphidq.subs(dummy_vars)
    dphidq_minus = dphidq_dummy.subs(subs_minus)

    lhs =sym.Matrix([p.T,H])
    rhs = sym.Matrix([λ*dphidq_minus,0])
    impact_eqs = sym.Eq(lhs,rhs)
    return impact_eqs

#Checking for Impacts
def impact_condition(Phi,s,threshold=1e-6):
    phi_func = sym.lambdify(q,Phi)
    phi_val = phi_func(s[0],s[1],s[2],s[3],s[4],s[5])
    return phi_val < threshold

#Finding velocities after impact
def impact_update(Phi,s):
    more_dummys = {x_b_d:s[0],y_b_d:s[1],th_b_d:s[2],x_j_d:s[3],y_j_d:s[4],th_j_d:s[5],
                   x_bdot_m:s[6],y_bdot_m:s[7],th_bdot_m:s[8],x_jdot_m:s[9],y_jdot_m:s[10],th_jdot_m:s[11]}

    impact_subs = impact_equations(Phi).subs(more_dummys)
    qdot_plus = sym.Matrix([x_bdot_p, y_bdot_p, th_bdot_p, x_jdot_p, y_jdot_p, th_jdot_p,λ])

    solutions = sym.solve(impact_subs, qdot_plus, dict=True)
    found_lam = None
    for soln in solutions:
        if abs(soln[λ]) > 1e-06:
            found_lam = soln
            break
    if found_lam is None:
        raise ValueError("No Impact")
    return [
        s[0],s[1],s[2],s[3],s[4],s[5],
        found_lam[x_bdot_p], found_lam[y_bdot_p], found_lam[th_bdot_p],
        found_lam[x_jdot_p], found_lam[y_jdot_p], found_lam[th_jdot_p]
    ]

#simulating with the impact update
def simulate_jack_box(f,x0,tspan,dt,integrate):
  N = int((max(tspan)-min(tspan))/dt)
  x = np.copy(x0)
  tvec = np.linspace(min(tspan),max(tspan),N)
  xtraj = np.zeros((len(x0),N))
  step = 0
  while step<N:
    for (index,checker) in enumerate(phi_list):
        if impact_condition(checker, x):
          step = step-2
          x = np.copy(xtraj[:,step])
          xtraj[:,step] = impact_update(checker,x)
          x = np.copy(xtraj[:,step])
          break
    else:
      xtraj[:,step] = integrate(f, x, dt, tvec[step])
      x = np.copy(xtraj[:,step])
    step = step+1
  return xtraj

def dynamic_jack(s,t):
    state_index = s[:12]
    return np.array([s[6], s[7], s[8], s[9], s[10], s[11],
                     x_b_num(*state_index,t), y_b_num(*state_index,t), th_b_num(*state_index,t),
                     x_j_num(*state_index,t), y_j_num(*state_index,t), th_j_num(*state_index,t)])

#initial conditions, keep the jack a bit above the box and angle the box
s0 = np.array([0,0,np.pi/3,0,1,0,0,0,0,0,0,0])
tspan = [0,10]  #10 seconds
dt = 0.01  ##small enoguh time step
#Run simulation
traj = simulate_jack_box(dynamic_jack,s0,tspan,dt,integrate)
#plotting
time = np.linspace(tspan[0],tspan[1], traj.shape[1])
plt.figure(figsize=(12,8))
plt.plot(time,traj[0],label='x_box')
plt.plot(time,traj[1],label='y_box')
plt.plot(time,traj[2],label='theta_box',color='purple')
plt.plot(time,traj[3],label='x_jack')
plt.plot(time,traj[4],label='y_jack',color='r')
plt.plot(time,traj[5],label='theta_jack',color='b')
plt.xlabel("Time (sec)")
plt.ylabel("Theta (Rads)")
plt.title("Jack in Box System over Time")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

def animate_jack_in_box(traj_array, L_box=3,L_jack=0.5, T=10):
    """
    Function to generate web-based animation of double-pendulum system

    Parameters:
    ================================================
    theta_array:
        trajectory of theta1 and theta2, should be a NumPy array with
        shape of (2,N)
    L1:
        length of the first pendulum
    L2:
        length of the second pendulum
    T:
        length/seconds of animation duration

    Returns: None
    """
    ################################
    # Imports required for animation.
    from plotly.offline import init_notebook_mode, iplot
    from IPython.display import display, HTML
    import plotly.graph_objects as go

    #######################
    # Browser configuration.
    def configure_plotly_browser_state():
        import IPython
        display(IPython.core.display.HTML('''
            <script src="/static/components/requirejs/require.js"></script>
            <script>
              requirejs.config({
                paths: {
                  base: '/static/base',
                  plotly: 'https://cdn.plot.ly/plotly-1.5.1.min.js?noext',
                },
              });
            </script>
            '''))
    configure_plotly_browser_state()
    init_notebook_mode(connected=False)

    ###############################################
    # Getting data from pendulum angle trajectories.
    # xx1=L1*np.sin(theta_array[0])
    # yy1=-L1*np.cos(theta_array[0])
    # xx2=xx1+L2*np.sin(theta_array[0]+theta_array[1])
    # yy2=yy1-L2*np.cos(theta_array[0]+theta_array[1])
    N = len(traj_array[0]) # Need this for specifying length of simulation
    ###############################################
    # Define arrays containing data for frame axes
    # Set arrays to be teh corners of the box
    box_BL = np.array([-L_box/2, -L_box/2])
    box_BR = np.array([L_box/2, -L_box/2])
    box_TR = np.array([L_box/2, L_box/2])
    box_TL = np.array([-L_box/2, L_box/2])
    jack_bottom = np.array([0,-L_jack/2])
    jack_right = np.array([L_jack/2,0])
    jack_top = np.array([0,L_jack/2])
    jack_left = np.array([-L_jack/2,0])
    box_corner1 = np.zeros((2,N))
    box_corner2 = np.zeros((2,N))
    box_corner3 = np.zeros((2,N))
    box_corner4 = np.zeros((2,N))
    mass_b = np.zeros((2,N))
    mass_r = np.zeros((2,N))
    mass_t = np.zeros((2,N))
    mass_l = np.zeros((2,N))

    for i in range(N): # iteration through each time step
        # evaluate homogeneous transformation
        #positions and translations for box and jack
        t_WBox = np.array([[1,0,0,traj_array[0][i]],[0,1,0,traj_array[1][i]],[0,0,1,0],[0,0,0,1]])
        r_WBox = np.array([[np.cos(traj_array[2][i]),-np.sin(traj_array[2][i]),0,0],[np.sin(traj_array[2][i]),np.cos(traj_array[2][i]),0,0],[0,0,1,0],[0, 0,0,1]])
        t_WJack = np.array([[1,0,0,traj_array[3][i]],[0,1,0,traj_array[4][i]],[0,0,1,0],[0,0,0,1]])
        r_WJack = np.array([[np.cos(traj_array[5][i]), -np.sin(traj_array[5][i]), 0, 0], [np.sin(traj_array[5][i]), np.cos(traj_array[5][i]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        #convert frame into the world frame
        g_Wbox = t_WBox.dot(r_WBox)
        g_WJack = t_WJack.dot(r_WJack)

        #transform box/jack to world frame
        box_corner1[:,i] = g_Wbox.dot([box_BL[0],box_BL[1],0,1])[0:2]
        box_corner2[:,i] = g_Wbox.dot([box_BR[0],box_BR[1],0,1])[0:2]
        box_corner3[:,i] = g_Wbox.dot([box_TR[0],box_TR[1],0,1])[0:2]
        box_corner4[:,i] = g_Wbox.dot([box_TL[0],box_TL[1],0,1])[0:2]
        mass_b[:,i] = g_WJack.dot([jack_bottom[0],jack_bottom[1],0,1])[0:2]
        mass_r[:,i] = g_WJack.dot([jack_right[0],jack_right[1],0,1])[0:2]
        mass_t[:,i] = g_WJack.dot([jack_top[0],jack_top[1],0,1])[0:2]
        mass_l[:,i] = g_WJack.dot([jack_left[0],jack_left[1],0,1])[0:2]

   ####################################
    # Using these to specify axis limits.
    xm = np.min(box_corner1)-0.5
    xM = np.max(box_corner2)+0.5
    ym = np.min(box_corner3)-2.5
    yM = np.max(box_corner4)+1.5

    ###########################
    # Defining data dictionary.
    # Trajectories are here.
    data=[
        # note that except for the trajectory (which you don't need this time),
        # you don't need to define entries other than "name". The items defined
        # in this list will be related to the items defined in the "frames" list
        # later in the same order. Therefore, these entries can be considered as
        # labels for the components in each animation frame
        dict(name = 'Box'),
        dict(name = 'Jackarm1'),
        dict(name = 'Jackarm2'),
        dict(name = 'Mass E'),
        dict(name = 'Mass F'),
        dict(name = 'Mass G'),
        dict(name = 'Mass H'),]
        # You don't need to show trajectory this time,
        # but if you want to show the whole trajectory in the animation (like what
        # you did in previous homeworks), you will need to define entries other than
        # "name", such as "x", "y". and "mode".

        # dict(x=xx1, y=yy1,
        #      mode='markers', name='Pendulum 1 Traj',
        #      marker=dict(color="fuchsia", size=2)
        #     ),
        # dict(x=xx2, y=yy2,
        #      mode='markers', name='Pendulum 2 Traj',
        #      marker=dict(color="purple", size=2)
        #     ),
    ################################
    # Preparing simulation layout.
    # Title and axis ranges are here.
    layout=dict(autosize=False, width=1000, height=1000,
                xaxis=dict(range=[xm, xM], autorange=False, zeroline=False,dtick=1),
                yaxis=dict(range=[ym, yM], autorange=False, zeroline=False,scaleanchor = "x",dtick=1),
                title='Jack In The Box Simulator',
                hovermode='closest',
                updatemenus= [{'type': 'buttons',
                               'buttons': [{'label': 'Play','method': 'animate',
                                            'args': [None, {'frame': {'duration': T, 'redraw': False}}]},
                                           {'args': [[None], {'frame': {'duration': T, 'redraw': False}, 'mode': 'immediate',
                                            'transition': {'duration': 0}}],'label': 'Pause','method': 'animate'}
                                          ]
                              }]
               )
    ########################################
    # Defining the frames of the simulation.
    frames=[dict(data=[# first three objects correspond to the arms and two masses,
                       # same order as in the "data" variable defined above (thus
                       # they will be labeled in the same order)
                       dict(x=[box_corner1[0][k],box_corner2[0][k],box_corner3[0][k],box_corner4[0][k],box_corner1[0][k]],
                            y=[box_corner1[1][k],box_corner2[1][k],box_corner3[1][k],box_corner4[1][k],box_corner1[1][k]],
                            mode='lines',
                            line=dict(color='black',width=5),
                            ),
                       dict(x=[mass_l[0][k],mass_r[0][k]],y=[mass_l[1][k],mass_r[1][k]],
                            mode='lines',
                            line=dict(color='blue',width=3),
                            ),
                       dict(x=[mass_b[0][k],mass_t[0][k]],y=[mass_b[1][k],mass_t[1][k]],
                            mode='lines',
                            line=dict(color='blue',width=3),
                            ),
                       go.Scatter(x=[mass_b[0][k]],y=[mass_b[1][k]],
                            mode='markers',
                            marker=dict(color="red",size=11)),
                       go.Scatter(x=[mass_r[0][k]],y=[mass_r[1][k]],
                            mode='markers',
                            marker=dict(color="orange",size=11)),
                       go.Scatter(x=[mass_t[0][k]],y=[mass_t[1][k]],
                            mode='markers',
                            marker=dict(color="violet",size=11)),
                       go.Scatter(x=[mass_l[0][k]],y=[mass_l[1][k]],
                            mode='markers',
                            marker=dict(color="cyan",size=11)),
                      ]) for k in range(N)]

    #######################################
    # Putting it all together and plotting.
    figure1=dict(data=data, layout=layout, frames=frames)
    iplot(figure1)

animate_jack_in_box(traj,L_box=3,L_jack=0.5, T=10)
