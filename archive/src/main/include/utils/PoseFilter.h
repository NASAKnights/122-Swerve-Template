#include <frc/geometry/Pose3d.h>
#include <deque>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

class PoseFilter
{
public:
    PoseFilter(size_t maxSize, double positionTolerance, double rotationTolerance)
        : maxSize_(maxSize), positionTolerance_(positionTolerance), rotationTolerance_(rotationTolerance) {}

    /**
     * Checks if a new Pose3d is valid before adding it to the history queue.
     *
     * @param newPose The new pose to evaluate.
     * @return True if the pose is stable and can be used, False otherwise.
     */
    [[nodiscard]] bool IsPoseValid(const frc::Pose3d &newPose, const double time)
    {

        // Ignore duplicate timestamps or very close timestamps
        if (lastTimestamp_ && std::abs(time - *lastTimestamp_) < 1e-6)
        {
            return false;
        }
        Eigen::Matrix4d newTransform = newPose.ToMatrix();
        // Store the first pose, but don't use it for calculations
        if (!lastPose_)
        {
            lastPose_ = newTransform;
            return false;
        }

        Eigen::Matrix4d prevTransform = *lastPose_;

        // Compute translation difference
        Eigen::Vector3d prevTranslation = prevTransform.block<3, 1>(0, 3);
        Eigen::Vector3d newTranslation = newTransform.block<3, 1>(0, 3);
        double translationError = (newTranslation - prevTranslation).norm();

        // Compute rotation difference
        Eigen::Matrix3d prevRotation = prevTransform.block<3, 3>(0, 0);
        Eigen::Matrix3d newRotation = newTransform.block<3, 3>(0, 0);
        bool rotationApprox = prevRotation.isApprox(newRotation, rotationTolerance_);

        // If a large jump is detected, reset and wait for stabilization
        if (translationError > positionTolerance_ || !rotationApprox)
        {
            Reset();
            return false;
        }

        // Add new pose to queue
        poses_.push_back(newTransform);
        lastPose_ = newTransform; // Update last pose reference

        // Maintain FIFO queue size
        if (poses_.size() > maxSize_)
        {
            poses_.pop_front();
            return true; // Only mark stable after the queue is full
        }

        return false;
    }

    /**
     * Resets the filter, clearing all stored poses.
     */
    void Reset()
    {
        poses_.clear();
        lastPose_.reset();
        lastTimestamp_.reset();
    }

    // /**
    //  * Returns whether the filter has reached stability.
    //  * @return True if the queue is full, False otherwise.
    //  */
    // [[nodiscard]] bool IsStable() const { return stable_; }

private:
    std::deque<Eigen::Matrix4d> poses_;       // Stores recent pose history
    std::optional<Eigen::Matrix4d> lastPose_; // Last valid pose
    size_t maxSize_;                          // Number of poses to store before stability
    double positionTolerance_;                // Position threshold for jumps
    double rotationTolerance_;                // Rotation threshold for jumps
    std::optional<double> lastTimestamp_;     // Last recorded timestamp
};
