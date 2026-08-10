#include <raylib.h>
#include "rlgl.h"
#include <raymath.h>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <deque>
#include <tuple>
#include <algorithm>

using namespace std;

/*
typedef struct Matrix {
    float m0, m4, m8, m12;  // Row 1: x-axis basis vector X, translation X
    float m1, m5, m9, m13;  // Row 2: y-axis basis vector Y, translation Y
    float m2, m6, m10, m14; // Row 3: z-axis basis vector Z, translation Z
    float m3, m7, m11, m15; // Row 4: Homogeneous coordinates (usually 0, 0, 0, 1)
} Matrix;
*/

Matrix rotate (Matrix mat, Vector3 right, Vector3 up, Vector3 forward) {
    Matrix ret = mat;
    tie(ret.m0, ret.m1, ret.m2) = {right.x, right.y, right.z};
    tie(ret.m4, ret.m5, ret.m6) = {up.x, up.y, up.z};
    tie(ret.m8, ret.m9, ret.m10) = {forward.x, forward.y, forward.z};
    return ret;
}

Matrix translate (Matrix mat, Vector3 translation) {
    Matrix ret = mat;
    tie(ret.m12, ret.m13, ret.m14) = {translation.x, translation.y, translation.z};
    return ret;
}

struct EulerAngle {
    float pitch{}, yaw{}, roll{};
    EulerAngle() {
        pitch = -45.0f;
        yaw = 0.0f; // 0 degrees with the -ve z axis, into the screen
    }

    Vector3 toVector() {
        float vx = cos(yaw * DEG2RAD) * cos(pitch * DEG2RAD);
        float vy = sin(pitch * DEG2RAD);
        float vz = sin(yaw * DEG2RAD) * cos(pitch * DEG2RAD);

        return (Vector3) {vx, vy, vz};
    }
};

// Conflicts with raylib's `Quanterion`.
struct Quaternion_def {
    float w, x, y, z;

    // theta is in degrees.
    Quaternion_def(){
        w = 1;
        x = 0;
        y = 0;
        z = 0;
    }
    Quaternion_def(float w, float x, float y, float z) {
        tie (this->w, this->x, this->y, this->z) = {w, x, y, z};
    }
    Quaternion_def(const Vector3& n, float theta) {
        theta = theta * DEG2RAD; // sin() and cos() use radian.
        
        w = cos(theta / 2.0f);
        auto v = n * sin(theta / 2.0f);
        tie(x, y, z) = {v.x, v.y, v.z};
    }

    Quaternion_def inverted() const {
        Quaternion_def q;

        q.w = w;
        q.x = -x;
        q.y = -y;
        q.z = -z;

        return q;
    }

    Quaternion_def multiply (Quaternion_def other) const {
        float wr = this->w, ws = other.w;
        Vector3 vr{x, y, z}, vs{other.x, other.y, other.z};

        float w_new = ws * wr + Vector3DotProduct(vr, vs);
        Vector3 v_new = vr * ws + vs * wr + Vector3CrossProduct(vr, vs);

        return Quaternion_def(w_new, v_new.x, v_new.y, v_new.z);
    }

    Vector3 transfrom_vector(Vector3 v) const {
        auto p = Quaternion_def(0, v.x, v.y, v.z);

        // p = this->multiply(p).multiply(this->inverted());
        p = this->multiply(p);
        p = p.multiply(this->inverted());

        return Vector3 {p.x, p.y, p.z};
    }

    static Quaternion_def slerp (Quaternion_def start_quat, 
        Quaternion_def final_quat, 
        float start_time,
        float end_time,
        float cur_time)
    {
        auto d = final_quat.multiply(start_quat.inverted());

        // No change
        if (final_quat.w == 1)
            return final_quat;
        // scale d
        Quaternion_def scaled_d;
        float theta = acos(d.w);

        if (abs(sin(theta)) <= 0.001f)
            return final_quat;

        float interpolated_theta = (cur_time - start_time) / (end_time - start_time) * theta;
        cout << "cur angle : " << interpolated_theta * RAD2DEG << ", target angle : " << theta * RAD2DEG << "\n";
        scaled_d.w = cos(interpolated_theta);
        
        Vector3 d_v = Vector3{d.x, d.y, d.z} / sin(theta) * sin(interpolated_theta);
        scaled_d.x = d_v.x;
        scaled_d.y = d_v.y;
        scaled_d.z = d_v.z;

        return start_quat.multiply(scaled_d);
    }

    void trace() {
        cout << "quat : " << w << ", " << x << ", " << y << ", " << z << "\n";
    }
};

struct AABB {
    Vector3 vec_min, vec_max;
    AABB() {}

    AABB(Vector3 mn, Vector3 mx) {
        vec_min = mn;
        vec_max = mx;
    }
};

struct Enemy {
    Vector3 center;
    AABB bbox;
    float yaw_angle;
    int health = 3;
    Vector3 scaling {1, 1, 1};

    Enemy(Vector3 c, float _yaw_angle) {
        center = c;
        yaw_angle = _yaw_angle;
        bbox = AABB();
    }
    
    float start_rotation_time, end_rotation_time;
    void start_rotation (float cur_time) {
        start_rotation_time = cur_time;
        end_rotation_time = cur_time + 3;
    }

    float interpolated_rotation(float cur_time) {
        float rotation = (cur_time - start_rotation_time) / (end_rotation_time - start_rotation_time) * 360.0f;
        return rotation;
    }
};

struct Bullet {
    Vector3 center;
    float creation_time, destruction_time;
    float start_size = 0.3f, end_size = 0.3f;

    Bullet (Vector3 c, float c_time) {
        center = c;
        creation_time = c_time;
        destruction_time = c_time + 1000;
    }

    bool should_delete(float t) {
        return t > destruction_time;
    }

    Vector3 current_size(float t) {
        float interval_pos = (t - creation_time) / (destruction_time - creation_time);
        float sz = start_size + interval_pos * (end_size - start_size);

        return Vector3{sz, sz, sz};
    }
};

struct EntityTransform {
    Matrix _global_transform;
    Matrix _local_transform;

    EntityTransform* _move_parent = nullptr;

    Matrix getGlobalTransform() {
        if (_move_parent == nullptr)
            return _global_transform;
        return _local_transform * _move_parent->_global_transform;
    }

    Matrix getLocalTransform() {
        return _local_transform;
    }

    void setParent(EntityTransform* parent) {
        if (parent == nullptr) {
            // Update the global coordinate before breaking free
            _global_transform = getGlobalTransform();
            _move_parent = nullptr;
        } else {
            _local_transform = getGlobalTransform() * MatrixInvert(parent->getGlobalTransform());
            _move_parent = parent;
        }
    }

    void setGlobalTransform(Matrix new_global_transform) {
        if (_move_parent != nullptr) {
            _local_transform = new_global_transform * MatrixInvert(_move_parent->getGlobalTransform());
        } else {
            _global_transform = new_global_transform;
        }
    }

    void translate_item (Vector3 extra_translation) {
        Matrix translation_mat = MatrixTranslate(extra_translation.x, 
                                                    extra_translation.y,
                                                    extra_translation.z);
        if (_move_parent == nullptr) {
            setGlobalTransform(getGlobalTransform() * translation_mat);
        } else {
            _local_transform = getLocalTransform() * translation_mat;
        }
    }

    // for now item rotates about its center.
    void rotate_item(float theta) {

    }
};

struct Entity : public EntityTransform {
    Vector3 center;

    Vector3 getCenter() {
        return Vector3Transform({0, 0, 0}, getGlobalTransform());
    }

    Entity(Vector3 _center) {
        _global_transform = MatrixTranslate(_center.x, _center.y, center.z);
    }
};

struct MerryGoRound : public EntityTransform {
    Vector3 center; // draw as a circle
    Vector3 child_local_center = {2, 0.5, 2};
    float radius = 4.0f;

    float app_start_time;
    float rotation_interval = 3.0f; // in seconds.

    Matrix _m_child_transform = MatrixIdentity();

    float evaluate_angle () {
        float total_time = GetTime(); // app starts at 0 seconds.
        float rem_partial_rotation = total_time / rotation_interval - int(total_time / rotation_interval);

        return 360 * rem_partial_rotation;
    }

    void setCenter(Vector3 _center) {
        center = _center;
        setGlobalTransform(translate(MatrixIdentity(), _center));
        _m_child_transform = translate(MatrixIdentity(), child_local_center + center);
    }

    void rotateSlightly() {
        float degrees_per_frame = 360 / 60 * 0.07f; // x rotations per second

        Matrix mSpin = MatrixRotate({0, 1, 0}, degrees_per_frame * DEG2RAD);
        Matrix rotation_only = translate(_global_transform, {0, 0, 0});
        Matrix translation_only = translate(MatrixIdentity(), center);

        _m_child_transform = (_m_child_transform) * MatrixInvert(_global_transform) * mSpin * _global_transform;
        _global_transform = rotation_only * mSpin * translation_only;
    }
};

struct Plane {
    // ax + by + cz = d;
    Vector3 normal;
    float d;
};

struct Frustum {
    Plane top, bottom, left, right, near, far;
} camera_frustum;

Matrix constructCameraView(const Camera& camera) {
    Vector3 translation = camera.position;

    Vector3 forward = Vector3Normalize(camera.target - camera.position);

    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera.up)); // cross forward with ref up.
    Vector3 up = Vector3Normalize(Vector3CrossProduct(right, forward));

    Vector3 opengl_forward = Vector3Scale(forward, -1.0f);

    Matrix R_INV = MatrixTranspose(rotate(MatrixIdentity(), right, up, opengl_forward));
    
    Matrix fast_inverse = translate(R_INV, Vector3Transform(translation * -1, R_INV));

    return fast_inverse;
}

void evaluateFrustum(Vector3 front, 
    Vector3 camera_position, 
    EulerAngle camera_euler_angle,
    float fovy,
    float fovx);

bool inside_frustum(Vector3 center);

bool clip_line(int d, AABB& bbox, Vector3& v0, Vector3& v1, float& f_low, float& f_high);
bool line_AABB_intersection(AABB& bbox, Vector3& v0, Vector3& v1, Vector3& vecIntersection, float& flFraction);

void rotate_vector_around_axis(Vector3 n_hat, float theta, Vector3 v);

float infinity = 1e9;

void print_v (Vector3 v) {
    cout << "x : " << v.x << ", y : " << v.y << ", z : " << v.z << "\n";
}

void print_matrix(Matrix m) {
    cout << "printing a matrix : \n";
    cout << m.m0 << " " << m.m1 << " " << m.m2 << " " << m.m3 << "\n";
    cout << m.m4 << " " << m.m5 << " " << m.m6 << " " << m.m7 << "\n";
    cout << m.m8 << " " << m.m9 << " " << m.m10 << " " << m.m11 << "\n";
    cout << m.m12 << " " << m.m13 << " " << m.m14 << " " << m.m15 << "\n";
}

int main(void) {

    float width = 1200, height = 650;

    InitWindow(width, height, "mfgd - 39 - camera-view-transform");

    float l, w, h;
    l = w = h = 2;
    Entity player({0, h / 2.0f, 0});

    float enemy_l = 4, enemy_w = 4, enemy_h = 4; 

    vector<Enemy> enemies = {Enemy({-7, enemy_h / 2.0f, 4}, 45), 
                            Enemy({8, enemy_h / 2.0f, 8}, 72),
                            Enemy({8, enemy_h / 2.0f, -8}, 72)};

    enemies.back().scaling = {2, 1, 1};

    for (auto& e: enemies) {
        auto mn = Vector3Subtract(e.center, {enemy_l / 2.0f, enemy_w / 2.0f, enemy_h / 2.0f});
        auto mx = Vector3Add(e.center, {enemy_l / 2.0f, enemy_w / 2.0f, enemy_h / 2.0f});

        AABB b(mn, mx);
        e.bbox = b;
    }

    // "Manual" way for illustration. Do not use this instance.
    MerryGoRound merry_go_round;
    merry_go_round.center = {12, 2, -12};

    MerryGoRound merry_go_round_2;
    merry_go_round_2.setCenter({-12, -2, -12});

    EulerAngle angle;

    Camera3D camera = { 0 };
    {
        camera.position = player.getCenter() - angle.toVector() * 8.0;
        camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        SetTargetFPS(60);
        DisableCursor();
    }

    float max_pitch = 80.0f, min_pitch = -80.0f;
    float sensitivity = 0.2f;

    deque<pair<Vector3,Vector3>> lines;
    deque<Bullet> collision_cubes;

    float zoom = 12.0f;

    int collisions_count = 0;

    // slerping
    Vector3 clock_vec = {0, 0, 10}; // will rotate this around some axis
    // cout << "clock vec : " << clock_vec.x << ", " << clock_vec.y << ", " << clock_vec.z << "\n";

    float quat_start_time{}, quat_end_time{};
    Vector3 slerped = clock_vec;

    while (!WindowShouldClose()) {

        // mouse input
        {
            float zoom_change = GetMouseWheelMoveV().y * sensitivity;
            zoom += zoom_change;

            auto mouse_movement = GetMouseDelta();

            angle.pitch -= mouse_movement.y * sensitivity;
            {
                angle.pitch = max (min_pitch, angle.pitch);
                angle.pitch = min (max_pitch, angle.pitch);
            }
            angle.yaw += mouse_movement.x * sensitivity;

            if (angle.yaw > 360.0f) {
                angle.yaw -= 360.0f;
            } else if (angle.yaw < 0) {
                angle.yaw += 360.0f;
            }
        }

        Vector3 direction = angle.toVector() * sensitivity;

        auto ref_up = camera.up;
        auto forward = direction;
        auto forward_normalized = angle.toVector();
        forward_normalized.y = 0;
        forward_normalized = Vector3Normalize(forward_normalized);
        auto right_normalized = Vector3Normalize(Vector3CrossProduct(forward, ref_up));
        auto right = right_normalized * sensitivity;
        auto true_up = Vector3Normalize(Vector3CrossProduct(right, forward));

        // project onto the xz plane
        forward.y = 0;
        forward = Vector3Normalize(forward) * sensitivity;

        // keyboard input
        {
            // Must update this
            Vector3 extra_translation{0, 0, 0};
            if (IsKeyDown(KEY_W)) {
                extra_translation = forward;
            } if (IsKeyDown(KEY_S)) {
                extra_translation = Vector3Negate(forward);
            }
            
            if (IsKeyDown(KEY_D)) {
                extra_translation = right;
            } else if (IsKeyDown(KEY_A)) {
                extra_translation = Vector3Negate(right);
            }

            player.translate_item(extra_translation);
        }

        // Player uses transformation of the merry go round
        {
            auto v = player.getCenter() - merry_go_round_2.center;
            v.y = 0;

            auto dist = sqrt(Vector3DotProduct(v, v));

            if (dist <= merry_go_round_2.radius) {
                cout << "trace" << dist << " : i am INSIDE the merry go round\n";
                player.setParent(&merry_go_round_2);
            } else {
                cout << "trace" << dist << " : i am OUTSIDE the merry go round\n";
                player.setParent(nullptr);
            }
        }

        camera.position = player.getCenter() - angle.toVector() * zoom;
        camera.target = player.getCenter();

        Matrix cameraProjection = MatrixPerspective(camera.fovy * DEG2RAD, (float)GetScreenWidth()/(float)GetScreenHeight(), 0.01f, 1000.0f);
        Matrix cameraView = constructCameraView(camera);

        evaluateFrustum(direction, player.getCenter(), angle, 45, 45);

        // Move enemies towards player
        {
            // for (auto& e : enemies) {
            //     Vector3 d = Vector3Normalize(player.getCenter() - e.center);
            //     d *= 0.03;

            //     e.center += d;
            // }

            // // re-eval aabb
            // for (auto& e: enemies) {
            //     auto mn = Vector3Subtract(e.center, {enemy_l / 2.0f, enemy_w / 2.0f, enemy_h / 2.0f});
            //     auto mx = Vector3Add(e.center, {enemy_l / 2.0f, enemy_w / 2.0f, enemy_h / 2.0f});

            //     AABB b(mn, mx);
            //     e.bbox = b;
            // }
        }

        // detecting bullet collision
        {
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) || IsKeyDown(KEY_SPACE)) {
                lines.push_back({player.getCenter(), (player.getCenter() + Vector3Normalize(direction) * 500.0f)});
                if (lines.size() > 1) {
                    lines.pop_front();
                }

                float smallest_fraction = infinity;
                Vector3 nearest_intersection;

                Enemy* collided_enemy = nullptr;

                Vector3 intersection;
                float fraction;
                for (auto& e: enemies) {
                    auto to_origin = MatrixTranslate(-e.center.x, -e.center.y, -e.center.z);
                    auto scale = MatrixScale(e.scaling.x, e.scaling.y, e.scaling.z);
                    auto rotate = MatrixRotate({0, 1, 0}, e.yaw_angle * DEG2RAD);
                    auto to_location = MatrixTranslate(e.center.x, e.center.y, e.center.z);

                    Matrix final_transform = MatrixMultiply(to_origin, scale);
                    final_transform = MatrixMultiply(final_transform, MatrixMultiply(rotate, to_location));

                    Matrix inverse_transform = MatrixInvert(final_transform);

                    for (auto l: lines) {
                        Vector3 v0 = Vector3Transform(l.first, inverse_transform);
                        Vector3 v1 = Vector3Transform(l.second, inverse_transform);
                        if (line_AABB_intersection(e.bbox, v0, v1, intersection, fraction)) {
                            // cout << "trace : has collided : " << ++collisions_count << "\n";

                            // - transform `intersection` back to the world coordinates.
                            // - `fraction` should be the same regardless of whether the ray was transformed
                            // or not.
                            if (fraction < smallest_fraction) {
                                smallest_fraction = fraction;
                                // nearest_intersection = l.first + Vector3Subtract(l.second, l.first) * fraction;
                                nearest_intersection = Vector3Transform(intersection, final_transform);
                                collided_enemy = &e;
                            }
                        }
                    }
                }

                if (smallest_fraction != infinity) {
                    // cout << "trace : current time : " << GetTime() << "\n";
                    collision_cubes.push_back(Bullet(nearest_intersection, GetTime()));
                    collided_enemy->start_rotation(GetTime());

                    collided_enemy->health--;
                    if (collided_enemy->health <= 0) {
                        for (int i = 0; i < enemies.size(); ++i) {
                            if (&enemies[i] == collided_enemy) {
                                enemies.erase(enemies.begin() + i);
                                break;
                            }
                        }
                    }
                }

                // line-plane intersection - bullet collision with the grid.
                /*
                    (v0 + k * direction) is my vector
                    ax + by + cz + d = 0 is the plane equation where i should substitute into

                    dot({a, b, c}, v0) + k * dot({a, b, c}, direction) = d
                    
                    plane equation of the grid -> "1 * y = 0", a = 0, b = 1, c = 0, d = 0

                    k = -dot({0, 1, 0}, v0) / dot({0, 1, 0}, direction)
                    
                    check for zero denominator. no soln
                    check for -ve k, plane is in the opposite direction

                    Another way to look at it is that the ratio between 
                    camera position to a point in plane projected onto the plane normal
                    and
                    (k - unknown) * (camera position + direction) vector projected onto the plane normal

                    should be 1:1
                */

                Vector3 plane_vec = {0, 1, 0};
                
                auto numerator = -Vector3DotProduct(plane_vec, player.getCenter());
                auto denominator = Vector3DotProduct(plane_vec, Vector3Normalize(direction));
                if (denominator == 0) {
                    cout << "trace : line - plane collision: no solution\n";
                } else {
                    float k = numerator / denominator;
                    if (k < 0) {
                        cout << "trace : line - plane collision: opposite directions\n";
                    } else {
                        Vector3 intersection_pos = Vector3Add(player.getCenter(), Vector3Normalize(direction) * k);
                        collision_cubes.push_back(Bullet(intersection_pos, GetTime()));

                        cout << "trace : line - plane collision: collided at : " 
                        << intersection_pos.x << ", " << intersection_pos.y << ", " << intersection_pos.z 
                        << "\n";
                    }
                }
            }

            while (!collision_cubes.empty() && GetTime() > collision_cubes.front().destruction_time) {
                collision_cubes.pop_front();
            }
        }
        
        // test quaternions
        {
            // lines.clear();  
            // Vector3 p1 = {0, 5, 0};
            // Vector3 p2 = {0, 0, 5};
            // Quaternion_def q1({0, 1, 0}, 45);
            // Quaternion_def q2({1, 0, 0}, 0);
            // auto q = q1.multiply(q2);
            // lines.push_back({Vector3Zero(), p1});
            // lines.push_back({Vector3Zero(), p2});
            // lines.push_back({Vector3Zero(), q.transfrom_vector(p1)});
            // lines.push_back({Vector3Zero(), q.transfrom_vector(p2)});
        }

        // slerping
        {
            auto final_quat = Quaternion_def({0, 1, 0}, 360);
            auto start_quat = Quaternion_def(); // zero rotation quat
            auto slerped_quat = Quaternion_def();

            if (IsKeyPressed(KEY_Q)) {
                quat_start_time = GetTime();
                quat_end_time = quat_start_time + 1;

                slerped_quat = Quaternion_def::slerp(start_quat, 
                    final_quat, 
                    quat_start_time,
                    quat_end_time,
                    GetTime());

                cout << "trace : should rotate\n";
            } else if (GetTime() < quat_end_time) {
                slerped_quat = Quaternion_def::slerp(start_quat, 
                    final_quat, 
                    quat_start_time,
                    quat_end_time,
                    GetTime());

                cout << "trace : should rotate\n";

                slerped = slerped_quat.transfrom_vector(clock_vec);

                cout << "time : " << quat_start_time << "," << GetTime() << " : " << quat_end_time << "\n";
            }

            // slerped_quat.trace();

            // clock_vec = slerped_quat.transfrom_vector(clock_vec);
            slerped = slerped_quat.transfrom_vector(clock_vec);

            // cout << "clock vec : " << clock_vec.x << ", " << clock_vec.y << ", " << clock_vec.z << "\n";
        }
        
        // drawing
        {
            BeginDrawing();
                ClearBackground(RAYWHITE);

                // ## BeginMode3D(camera); replaced with custom projection and view matrices
                {
                    rlDrawRenderBatchActive();      // Flush any remaining 2D sprites
                    rlMatrixMode(RL_PROJECTION);    // Switch to projection matrix stack
                    rlPushMatrix();                 // Save current 2D projection matrix
                    rlSetMatrixProjection(cameraProjection); // Load your custom projection

                    rlMatrixMode(RL_MODELVIEW);     // Switch to modelview matrix stack
                    rlPushMatrix();                 // Save current 2D modelview matrix
                    rlSetMatrixModelview(cameraView); // Load your CUSTOM VIEW MATRIX

                    rlEnableDepthTest();            // Turn on 3D depth testing
                }
                    // player
                    {
                        // make player face the direction of the forward vector
                        Matrix player_basis;
                        player_basis = 
                        {
                            right_normalized.x, ref_up.x, forward_normalized.x, 0,
                            right_normalized.y, ref_up.y, forward_normalized.y, 0,
                            right_normalized.z, ref_up.z, forward_normalized.z, 0,
                            0,                  0,        0,                    1.0f
                        };

                        rlPushMatrix();
                            rlTranslatef(player.getCenter().x, player.getCenter().y, player.getCenter().z);
                            rlMultMatrixf(MatrixToFloat(player_basis));

                            // next video?
                            // rlMultMatrixf(MatrixToFloat(player_basis * player.getGlobalTransform()));

                            // You can avoid this if you render at exactly vector3zero instead of player.getCenter().
                            // rlTranslatef(-player.getCenter().x, -player.getCenter().y, -player.getCenter().z);
                            
                            DrawCube(Vector3Zero(), 0.1f, 0.1f, 0.1f, RED);
                            DrawCubeWires(Vector3Zero(), l, l, l, GREEN);
                        rlPopMatrix();
                    }

                    // merry go round
                    {
                        // Normal way
                        {
                            // disc
                            rlPushMatrix();
                                rlTranslatef(merry_go_round.center.x, merry_go_round.center.y, merry_go_round.center.z);
                                rlRotatef(merry_go_round.evaluate_angle(), 0, 1, 0);
                                // DrawRectangle(Vector3Zero(), merry_go_round.radius, {1, 0, 0}, 90, BLUE);
                                DrawCubeWires(Vector3Zero(), merry_go_round.radius * 2,
                                    0.1,
                                    merry_go_round.radius * 2, 
                                    BLUE
                                );
                            rlPopMatrix();

                            // child object
                            rlPushMatrix();
                                auto relative_vector = merry_go_round.child_local_center;

                                rlTranslatef(merry_go_round.center.x, merry_go_round.center.y, merry_go_round.center.z);
                                rlRotatef(merry_go_round.evaluate_angle(), 0, 1, 0);
                                rlTranslatef(relative_vector.x, relative_vector.y, relative_vector.z);
                                DrawCube(Vector3Zero(), 0.2f, 0.2f, 0.2f, RED);
                            rlPopMatrix();
                        }

                        // Acuumulated rotation - as explained in MFGD
                        {
                            merry_go_round_2.rotateSlightly();

                            // disc
                            {
                                rlPushMatrix();
                                    rlMultMatrixf(MatrixToFloat(merry_go_round_2.getGlobalTransform()));
                                    DrawCubeWires(
                                        Vector3Zero(), 
                                        merry_go_round_2.radius * 2,
                                        0.1f,
                                        merry_go_round_2.radius * 2,
                                        VIOLET
                                    );
                                rlPopMatrix();
                            }

                            // child object
                            {
                                rlPushMatrix();
                                    rlMultMatrixf(MatrixToFloat(merry_go_round_2._m_child_transform));
                                    DrawCube(Vector3Zero(), 0.5f, 0.5f, 0.5f, RED);
                                rlPopMatrix();
                            }
                        }
                    }
                    
                    // banner
                    {
                        rlPushMatrix();
                            Vector3 banner = {-8.0f, 0.0f, -8.0f};

                            // 1. Calculate the 2D direction vector from the banner to the player
                            float deltaX = player.getCenter().x - banner.x;
                            float deltaZ = player.getCenter().z - banner.z;

                            // 2. Use atan2f to get the precise 360-degree angle.
                            // We use deltaX and deltaZ relative to the grid. 
                            // You may need to add or subtract 90.0f depending on how your camera/textures align.
                            float banner_angle = atan2f(deltaX, deltaZ) * RAD2DEG;
                            rlTranslatef(banner.x, banner.y, banner.z);
                            rlRotatef(banner_angle, 0.0f, 1.0f, 0.0f);
                            rlTranslatef(-banner.x, -banner.y, -banner.z);

                            DrawCube(banner, 5.0f, 5.0f, 0.5f, VIOLET);
                            DrawCubeWires(banner, 5.0f, 5.0f, 0.5f, DARKPURPLE);
                        rlPopMatrix();
                    }
                        
                    DrawGrid(40, 1.0f);

                    // clock
                    {
                        DrawLine3D(Vector3Zero(), slerped, RED);
                    }

                    // bullets
                    {
                        for (auto l: lines) {
                            DrawLine3D(l.first, l.second, RED);
                        }

                        for (auto c: collision_cubes) {
                            float t = GetTime();
                            Vector3 sz = c.current_size(t);
                            DrawCube(c.center, sz.x, sz.y, sz.z, MAGENTA);
                        }

                        // bboxes
                        for (auto e : enemies) {
                            DrawCube(e.bbox.vec_min, 0.5f, 0.5f, 0.5f, RED);
                            DrawCube(e.bbox.vec_max, 0.5f, 0.5f, 0.5f, RED);
                        }
                    }

                    // enemies
                    {
                        /*
                            1. draw opaque objects.
                            2. sort transparent objects based on how close to the camera they are.
                            3. Draw transpaerent objects.
                        */
                        sort(enemies.begin(), enemies.end(), [&camera](Enemy e1, Enemy e2) {
                            float d1 = Vector3Distance(camera.position, e1.center);
                            float d2 = Vector3Distance(camera.position, e2.center);
                            return d1 > d2;
                        });

                        for (int i = 0 ; i < enemies.size(); ++i) {
                            // DrawCube(e, enemy_l, enemy_l, enemy_l, YELLOW);
                            auto e = enemies[i];

                            if (!inside_frustum(e.center))
                                continue;
                            
                            float cur_time = GetTime();
                            if (cur_time <= e.end_rotation_time) {
                                rlPushMatrix();
                                    rlTranslatef(e.center.x, e.center.y, e.center.z);
                                    float total_rotation = e.interpolated_rotation(cur_time) + e.yaw_angle;
                                    if (total_rotation >= 360)
                                        total_rotation -= 360;
                                    float theta = total_rotation * DEG2RAD;
                                    Matrix enemy_rotation = {
                                        cos(theta), 0, sin(theta), 0,
                                        0, 1, 0, 0,
                                        -sin(theta), 0, cos(theta), 0,
                                        0, 0, 0, 1
                                    };
                                    rlMultMatrixf(MatrixToFloat(enemy_rotation));

                                    rlScalef(e.scaling.x, e.scaling.y, e.scaling.z);

                                    rlTranslatef(-e.center.x, -e.center.y, -e.center.z);

                                    DrawCube(e.center, enemy_l, enemy_l, enemy_l, ColorAlpha(ORANGE, 0.5f));
                                    DrawCubeWires(e.center, enemy_l, enemy_l, enemy_l, BLUE);
                                rlPopMatrix();
                            } else {
                                rlPushMatrix();
                                    rlTranslatef(e.center.x, e.center.y, e.center.z);
                                    rlRotatef(e.yaw_angle, 0, 1, 0);
                                    rlScalef(e.scaling.x, e.scaling.y, e.scaling.z);
                                    rlTranslatef(-e.center.x, -e.center.y, -e.center.z);
                                    DrawCube(e.center, enemy_l, enemy_l, enemy_l, ColorAlpha(ORANGE, 0.5f));
                                    DrawCubeWires(e.center, enemy_l, enemy_l, enemy_l, BLUE);
                                rlPopMatrix();
                            }
                        }
                    }

                // ## EndMode3D(); replaced with custom projection and view matrices
                {
                    rlDrawRenderBatchActive();      // Flush 3D geometry before restoring matrices
                    rlDisableDepthTest();           // Turn off depth testing for 2D UI

                    rlMatrixMode(RL_PROJECTION);
                    rlPopMatrix();                  // Restore original 2D projection matrix

                    rlMatrixMode(RL_MODELVIEW);
                    rlPopMatrix();                  // Restore original 2D modelview matrix
                }

                // labels and stuff
                {
                    DrawFPS(10, 10);

                    string pitch = to_string((int)angle.pitch);
                    string yaw = to_string((int)angle.yaw);

                    DrawText(pitch.c_str(), 10, 30, 10, GREEN);
                    DrawText(yaw.c_str(), 10, 50, 10, GREEN);
                }
            EndDrawing();
        }
    }

    CloseWindow();

    return 0;
}

float eval_d_of_plane (Vector3 normal, Vector3 point_on_plane)
{
    return -(normal.x * point_on_plane.x + 
            normal.y * point_on_plane.y + 
            normal.z * point_on_plane.z);
}

void evaluateFrustum(Vector3 front, 
    Vector3 camera_position, 
    EulerAngle camera_euler_angle,
    float fovy,
    float fovx)
{
    front = Vector3Normalize(front);

    float near_dist = 1.0f, far_dist = 100.0f;

    auto& near = camera_frustum.near;
    auto& far = camera_frustum.far;
    auto& top = camera_frustum.top;
    auto& bottom = camera_frustum.bottom;
    auto& left = camera_frustum.left;
    auto& right = camera_frustum.near;

    near.normal = front;
    near.d = eval_d_of_plane(near.normal, camera_position + front * near_dist);

    far.normal = Vector3Negate(front);
    far.d = eval_d_of_plane(far.normal, camera_position + front * far_dist);

    EulerAngle top_angle = camera_euler_angle;
    top_angle.pitch = camera_euler_angle.pitch - (90 - fovy);
    top.normal = top_angle.toVector();
    top.d = eval_d_of_plane(top.normal, camera_position);

    EulerAngle bottom_angle = camera_euler_angle;
    bottom_angle.pitch = camera_euler_angle.pitch + (90 - fovy);
    bottom.normal = bottom_angle.toVector();
    bottom.d = eval_d_of_plane(bottom.normal, camera_position);

    EulerAngle left_angle = camera_euler_angle;
    left_angle.yaw = camera_euler_angle.yaw + (90 - fovx);
    left.normal = left_angle.toVector();
    left.d = eval_d_of_plane(left.normal, camera_position);

    EulerAngle right_angle = camera_euler_angle;
    right_angle.yaw = camera_euler_angle.yaw - (90 - fovx);
    right.normal = right_angle.toVector();
    right.d = eval_d_of_plane(right.normal, camera_position);
}

bool inside_frustum(Vector3 center)
{
    vector<Plane> planes = {camera_frustum.near,
                            camera_frustum.far,
                            camera_frustum.top, 
                            camera_frustum.bottom,
                            camera_frustum.left,
                            camera_frustum.right};

    for (auto&p : planes) {
        float cur_d = eval_d_of_plane(p.normal, center);

        // my way: ax + by + cz = d, it approaches 0 as we get closer to origin
        if (cur_d > p.d)
            return false;

        // playlist
        // float d_of_center_along_normal = Vector3DotProduct(center, p.normal);
        // if (d_of_center_along_normal + p.d < 0)
        //     return false;
    }

    return true;
}

bool clip_line(AABB& bbox, Vector3& v0, Vector3& v1, float& f_low, float& f_high)
{
    float F1 = -infinity, F2 = infinity;
    {
        float p1x = bbox.vec_min.x, p2x = bbox.vec_max.x;
        float f1 = (p1x - v0.x) / (v1.x - v0.x);
        float f2 = (p2x - v0.x) / (v1.x - v0.x);

        if (f1 > f2)
            swap(f1, f2);

        F1 = max (F1, f1);
        F2 = min (F2, f2);
    }
    
    {
        float p1y = bbox.vec_min.y, p2y = bbox.vec_max.y;
        float f1 = (p1y - v0.y) / (v1.y - v0.y);
        float f2 = (p2y - v0.y) / (v1.y - v0.y);

        if (f1 > f2)
            swap(f1, f2);

        F1 = max (F1, f1);
        F2 = min (F2, f2);
    }
    {
        float p1z = bbox.vec_min.z, p2z = bbox.vec_max.z;
        float f1 = (p1z - v0.z) / (v1.z - v0.z);
        float f2 = (p2z - v0.z) / (v1.z - v0.z);

        if (f1 > f2)
            swap(f1, f2);

        F1 = max (F1, f1);
        F2 = min (F2, f2);
    }
    
    if (F1 > F2 || F2 < 0 || F1 > 1)
        return false;

    f_low = F1 < 0.0 ? 0.0 : F1;
    f_high = F2;

    if (f_low < 0.0) {
        cout << "trace : low f_low : " << f_low << ", f_high : " << f_high << "\n";
    }

    return true;
}

bool line_AABB_intersection(AABB& bbox, Vector3& v0, Vector3& v1, Vector3& vecIntersection, float& flFraction)
{
    float f_low = 0, f_high = 1;

    if (clip_line(bbox, v0, v1, f_low, f_high))
    {
        flFraction = f_low;
        
        Vector3 b = Vector3Subtract(v1, v0);
        vecIntersection = Vector3Add(v0, b * f_low);
        // vecIntersection = Vector3Add(v0, b * f_high);

        return true;
    }

    return false;
}

void rotate_vector_around_axis(Vector3 n_hat, float theta, Vector3 v) {
    auto p = n_hat * Vector3DotProduct(v, n_hat);
    auto e = Vector3Subtract(v, p);

    auto f = Vector3CrossProduct(e, n_hat);

    auto theta_in_radian = theta * DEG2RAD;
    auto e_dash = f * sin(theta_in_radian) + e * cos(theta_in_radian);

    auto v_dash = p + e_dash;
}

/*



*/