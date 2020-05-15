#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

class TmPoseConversion
{
public:
    ////////////////////////////////
    // tf::Transform
    ////////////////////////////////

    static inline void tf_from_vec(tf::Transform &T, const std::vector<double> &vec)
    {
        if (vec.size() != 6) return;
        T.setOrigin(tf::Vector3(vec[0], vec[1], vec[2]));
        tf::Quaternion quat;
        quat.setRPY(vec[3], vec[4], vec[5]);
        T.setRotation(quat);
    }
    static inline void vec_from_tf(std::vector<double> &vec, const tf::Transform &T)
    {
        const tf::Vector3 &P = T.getOrigin();
        tf::Matrix3x3 rot(T.getRotation());
        double r, p, y;
        rot.getRPY(r, p, y);
        vec.resize(6);
        vec[0] = P[0];
        vec[1] = P[1];
        vec[2] = P[2];
        vec[3] = r;
        vec[4] = p;
        vec[5] = y;
    }

    ////////////////////////////////
    // geometry_msgs::Pose
    ////////////////////////////////

    static inline void quat_msg_from_rpy(geometry_msgs::Quaternion &msg, double r, double p, double y)
    {
        tf::Quaternion quat;
        quat.setRPY(r, p, y);
        tf::quaternionTFToMsg(quat, msg);
    }
    static inline void quat_msg_from_rpy(geometry_msgs::Quaternion &msg, const std::vector<double> &vec)
    {
        if (vec.size() == 3) quat_msg_from_rpy(msg, vec[0], vec[1], vec[2]);
    }
    static inline void msg_from_vec(geometry_msgs::Pose &msg, const std::vector<double> &vec)
    {
        if (vec.size() != 6) return;
        msg.position.x = vec[0];
        msg.position.y = vec[1];
        msg.position.z = vec[2];
        quat_msg_from_rpy(msg.orientation, vec[3], vec[4], vec[5]);
    }
    static inline void rpy_from_quat_msg(double &r, double &p, double &y, const geometry_msgs::Quaternion &msg)
    {
        tf::Matrix3x3 rot(tf::Quaternion(msg.x, msg.y, msg.z, msg.w));
        rot.getRPY(r, p, y);
    }
    static inline void rpy_from_quat_msg(std::vector<double> &vec, const geometry_msgs::Quaternion &msg)
    {
        double r, p, y;
        rpy_from_quat_msg(r, p, y, msg);
        vec.resize(3);
        vec[0] = r;
        vec[1] = p;
        vec[2] = y;
    }
    static inline void vec_from_msg(std::vector<double> &vec, const geometry_msgs::Pose &msg)
    {
        double r, p, y;
        rpy_from_quat_msg(r, p, y, msg.orientation);
        vec.resize(6);
        vec[0] = msg.position.x;
        vec[1] = msg.position.y;
        vec[2] = msg.position.z;
        vec[3] = r;
        vec[4] = p;
        vec[5] = y;
    }

};