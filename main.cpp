#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"


constexpr auto stepsPerRound = 200;
constexpr auto stepRadians= 2 * 3.141592654 / stepsPerRound;

struct Stepper {
   std::atomic_int32_t steps;
};

class Mover : public rclcpp::Node
{
public:
    //using JointState = sensor_msgs::msg::JointState;

    using JointTrajectory = trajectory_msgs::msg::JointTrajectory;

    Mover() : Node("mover"),
        m_publisher{create_publisher<JointTrajectory>("set_joint_trajectory", 10)}
    {
        initJointState({"pan", "tilt"});

        m_publisherThread = std::make_unique<std::thread>(&Mover::publishLoop, this);
    }

    ~Mover() override
    {
        m_stop = true;
        if (m_publisherThread->joinable())
        {
            m_publisherThread->join();
        }
    }

    void move(size_t jointNumber, int steps)
    {
        m_steppers[jointNumber]->steps.store(steps);
    }

private:
    void initJointState(const std::vector<std::string> & jointNames)
    {

        m_jointTrajectory.header.frame_id = "world";

        m_jointTrajectory.joint_names = jointNames;

        const auto jointsSize = jointNames.size();

        m_jointTrajectory.points.resize(1);
        m_jointTrajectory.points[0].positions.resize(jointsSize);

        m_steppers.reserve(jointsSize);
        for(size_t i = 0; i < jointsSize; ++i)
        {
            m_steppers.push_back(std::make_unique<Stepper>());
        }
    }

    bool rotate()
    {
        bool result = false;
        for (size_t i = 0 ; i < m_steppers.size(); ++i)
        {
            auto& stepsStore = m_steppers[i]->steps;

            auto steps = stepsStore.load();
            if (steps > 0)
            {
                m_jointTrajectory.points[0].positions[i] += stepRadians;
                stepsStore.store(steps - 1);
                result = true;
            }
            else if (steps < 0)
            {
                m_jointTrajectory.points[0].positions[i] -= stepRadians;
                stepsStore.store(steps + 1);
                result = true;
            }
        }
        return result;

    }

    void publishLoop()
    {
        m_stop = false;
        size_t loopCounter = 0;
        while(!m_stop)
        {
            if(loopCounter == 20)
            {
                loopCounter = 0;
            }

            if(rotate() || loopCounter == 0)
            {
                //m_jointTrajectory.header.stamp = get_clock()->now();
                m_publisher->publish(m_jointTrajectory);
            }
            ++loopCounter;
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }
    }

private:
    std::shared_ptr<rclcpp::Publisher<JointTrajectory>> m_publisher;
    JointTrajectory m_jointTrajectory;
    std::unique_ptr<std::thread> m_publisherThread;
    std::vector<std::unique_ptr<Stepper>> m_steppers;
    std::atomic_bool m_stop;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    Mover mover{};

    while(true)
    {
        size_t jointNumber;
        int steps;
        std::cin >> jointNumber >> steps;
        if(jointNumber < 2)
        {
            mover.move(jointNumber, steps);
        }
        else
        {
            break;
        }
    }
    rclcpp::shutdown();
    return 0;
}
