import numpy as np
import matplotlib.pyplot as plt

def quintic_coeffs(q0, v0, a0, qf, vf, af, T: float) -> np.ndarray:
    q0 = np.atleast_1d(q0).astype(float)
    v0 = np.atleast_1d(v0).astype(float)
    a0_ = np.atleast_1d(a0).astype(float)
    qf = np.atleast_1d(qf).astype(float)
    vf = np.atleast_1d(vf).astype(float)
    af = np.atleast_1d(af).astype(float)
    shape = np.broadcast(q0, v0, a0_, qf, vf, af).shape
    q0, v0, a0_, qf, vf, af = (np.broadcast_to(arr, shape)
                               for arr in (q0, v0, a0_, qf, vf, af))
    coeffs = np.zeros(shape + (6,), dtype=float)
    coeffs[..., 0] = q0
    coeffs[..., 1] = v0
    coeffs[..., 2] = a0_ / 2.0

    T1, T2, T3, T4, T5 = T, T**2, T**3, T**4, T**5
    M = np.array([[T3,  T4,   T5],
                  [3*T2, 4*T3, 5*T4],
                  [6*T1, 12*T2,20*T3]], dtype=float)
    Minv = np.linalg.inv(M)

    r1 = qf - (coeffs[...,0] + coeffs[...,1]*T1 + coeffs[...,2]*T2)
    r2 = vf - (coeffs[...,1] + 2*coeffs[...,2]*T1)
    r3 = af - (2*coeffs[...,2])
    rhs = np.stack([r1, r2, r3], axis=-1)  # shape (...,3)
    sol = np.einsum('ij,...j->...i', Minv, rhs)
    coeffs[..., 3:] = sol
    return coeffs

def sample_quintic(coeffs: np.ndarray, t: np.ndarray):
    t = np.asarray(t, dtype=float)
    T = np.vstack([np.ones_like(t), t, t**2, t**3, t**4, t**5])
    Td = np.vstack([np.zeros_like(t), np.ones_like(t), 2*t, 3*t**2, 4*t**3, 5*t**4])
    Tdd = np.vstack([np.zeros_like(t), np.zeros_like(t), 2*np.ones_like(t), 6*t, 12*t**2, 20*t**3])
    q   = np.einsum('...i,iN->...N', coeffs, T)
    qd  = np.einsum('...i,iN->...N', coeffs, Td)
    qdd = np.einsum('...i,iN->...N', coeffs, Tdd)
    return q, qd, qdd

# —— 下面是调用示例 —— #

# 1. 指定状态参数
q0, v0, a0 = 0.1, 0.0, 0   # 起始：位置0，速度0，加速度0
qf, vf, af = 0.2, 3, 0   # 结束：位置1，速度0，加速度0
T = 1.0 / 30                   # 总时间
dt = 1.0 / 400                   # 时间步长

# 2. 计算系数并采样
coeffs = quintic_coeffs(q0, v0, a0, qf, vf, af, T)
t = np.arange(0, T + dt, dt)
q, qd, qdd = sample_quintic(coeffs, t)

# 3. 可视化
plt.figure()
plt.plot(t, q.squeeze(), label='Position')
plt.xlabel('Time [s]')
plt.ylabel('Position')
plt.title('Quintic Trajectory: Position vs Time')
plt.grid(True)

plt.figure()
plt.plot(t, qd.squeeze(), label='Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Velocity')
plt.title('Quintic Trajectory: Velocity vs Time')
plt.grid(True)

plt.figure()
plt.plot(t, qdd.squeeze(), label='Acceleration')
plt.xlabel('Time [s]')
plt.ylabel('Acceleration')
plt.title('Quintic Trajectory: Acceleration vs Time')
plt.grid(True)

plt.show()
