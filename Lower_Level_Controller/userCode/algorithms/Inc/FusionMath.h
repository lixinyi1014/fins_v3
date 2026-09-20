#ifndef FUSION_MATH_H
#define FUSION_MATH_H
#include <math.h>

namespace lower_controller
{
namespace fusion_math
{

inline float Dot(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline float Norm(const float a[3])
{
    return sqrtf(Dot(a, a));
}
inline bool Finite(const float *a, unsigned n)
{
    for (unsigned i = 0; i < n; ++i)
        if (!isfinite(a[i]))
            return false;
    return true;
}
inline bool Normalize(float *a, unsigned n)
{
    float sum = 0;
    for (unsigned i = 0; i < n; ++i)
        sum += a[i] * a[i];
    if (!isfinite(sum) || sum < 1e-12f)
        return false;
    const float scale = 1.0f / sqrtf(sum);
    for (unsigned i = 0; i < n; ++i)
        a[i] *= scale;
    return true;
}
inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
inline float WrapPi(float x)
{
    const float pi = 3.14159265358979323846f;
    float y = fmodf(x + pi, 2 * pi);
    if (y < 0)
        y += 2 * pi;
    return y - pi;
}
inline void Skew(const float v[3], float s[3][3])
{
    s[0][0] = 0;
    s[0][1] = -v[2];
    s[0][2] = v[1];
    s[1][0] = v[2];
    s[1][1] = 0;
    s[1][2] = -v[0];
    s[2][0] = -v[1];
    s[2][1] = v[0];
    s[2][2] = 0;
}
inline void Rotate(const float r[3][3], const float in[3], float out[3])
{
    for (unsigned i = 0; i < 3; ++i)
        out[i] = Dot(r[i], in);
}
inline void QuaternionProduct(const float a[4], const float b[4], float out[4])
{
    float q[4] = {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
    for (unsigned i = 0; i < 4; ++i)
        out[i] = q[i];
}
inline void RotationVectorQuaternion(const float v[3], float q[4])
{
    const float angle = Norm(v), scale = angle < 1e-6f ? 0.5f : sinf(angle * 0.5f) / angle;
    q[0] = cosf(angle * 0.5f);
    for (unsigned i = 0; i < 3; ++i)
        q[i + 1] = v[i] * scale;
    Normalize(q, 4);
}
inline void QuaternionMatrix(const float q[4], float r[3][3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    r[0][0] = 1 - 2 * (y * y + z * z);
    r[0][1] = 2 * (x * y - w * z);
    r[0][2] = 2 * (x * z + w * y);
    r[1][0] = 2 * (x * y + w * z);
    r[1][1] = 1 - 2 * (x * x + z * z);
    r[1][2] = 2 * (y * z - w * x);
    r[2][0] = 2 * (x * z - w * y);
    r[2][1] = 2 * (y * z + w * x);
    r[2][2] = 1 - 2 * (x * x + y * y);
}
inline void EulerQuaternion(float roll, float pitch, float yaw, float q[4])
{
    const float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2),
                cy = cosf(yaw / 2), sy = sinf(yaw / 2);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}
inline void QuaternionEuler(const float q[4], float angles[3])
{
    float r[3][3];
    QuaternionMatrix(q, r);
    angles[0] = atan2f(r[2][1], r[2][2]);
    angles[1] = asinf(Clamp(-r[2][0], -1, 1));
    angles[2] = atan2f(r[1][0], r[0][0]);
}

/* Cholesky avoids explicit inversion. Cholesky 分解后解线性方程，不对小主元强行取倒数。
 * n<=6；输入为行优先矩阵。返回 false 时调用方拒绝本次更新，保留原状态。 */
inline bool Cholesky(const float *a, float *l, unsigned n)
{
    for (unsigned i = 0; i < n * n; ++i)
        l[i] = 0;
    for (unsigned i = 0; i < n; ++i)
        for (unsigned j = 0; j <= i; ++j)
        {
            float sum = a[i * n + j];
            if (!isfinite(sum) || fabsf(sum - a[j * n + i]) > 1e-5f * (1 + fabsf(sum)))
                return false;
            for (unsigned k = 0; k < j; ++k)
                sum -= l[i * n + k] * l[j * n + k];
            if (i == j)
            {
                if (!isfinite(sum) || sum <= 1e-12f)
                    return false;
                l[i * n + j] = sqrtf(sum);
            }
            else
                l[i * n + j] = sum / l[j * n + j];
        }
    return true;
}
inline void SolveCholesky(const float *l, const float *b, float *x, unsigned n)
{
    float y[6] = {};
    for (unsigned i = 0; i < n; ++i)
    {
        float sum = b[i];
        for (unsigned k = 0; k < i; ++k)
            sum -= l[i * n + k] * y[k];
        y[i] = sum / l[i * n + i];
    }
    for (int i = int(n) - 1; i >= 0; --i)
    {
        float sum = y[i];
        for (unsigned k = unsigned(i) + 1; k < n; ++k)
            sum -= l[k * n + unsigned(i)] * x[k];
        x[i] = sum / l[unsigned(i) * n + unsigned(i)];
    }
}

} // namespace fusion_math
} // namespace lower_controller
#endif
