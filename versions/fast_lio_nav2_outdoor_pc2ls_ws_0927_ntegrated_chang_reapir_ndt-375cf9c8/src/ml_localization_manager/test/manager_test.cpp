#include <gtest/gtest.h>
#include "ml_localization_manager/manager.hpp"

using Manager = ml_loc_mgr::LocalizationManager;
using Level = Manager::StatusInfo::Level;
using State = Manager::State;

class ManagerTest : public ::testing::Test {
protected:
    Manager manager{Manager::Config{}};
    Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();

    void Request() {
        Manager::StatusInfo status;
        status.level = Level::kInit;
        manager.OnStatus(status, 100.0);
        ASSERT_TRUE(manager.Tick(100.0).request_reloc);
        manager.MarkRelocRequested();
        manager.PushOdom(100.0, identity);
    }
    void Accept() {
        Request();
        std::string reason;
        ASSERT_TRUE(manager.OnRelocPose(identity, 100.0, 100.0, reason));
    }
    void SendAndAcknowledge() {
        ASSERT_TRUE(manager.MarkSetPoseSent());
        ASSERT_TRUE(manager.ConfirmSetPose());
        ASSERT_TRUE(manager.MarkStartTrackingSent());
    }
    void Complete() {
        Accept();
        SendAndAcknowledge();
        manager.ConfirmStartTracking(100.1);
        Manager::StatusInfo status;
        status.level = Level::kTracking;
        status.scan_stamp = 100.2;
        ASSERT_TRUE(manager.OnStatus(status, 100.2));
        ASSERT_TRUE(manager.StartupCompleted());
    }
};

TEST_F(ManagerTest, RequiresActiveRequest) {
    manager.PushOdom(100.0, identity);
    std::string reason;
    EXPECT_FALSE(manager.OnRelocPose(identity, 100.0, 100.0, reason));
}
TEST_F(ManagerTest, RejectsStaleFutureAndNonfinitePose) {
    Request();
    std::string reason;
    EXPECT_FALSE(manager.OnRelocPose(identity, 99.0, 100.0, reason));
    EXPECT_FALSE(manager.OnRelocPose(identity, 101.0, 100.0, reason));
    EXPECT_FALSE(manager.OnRelocPose(identity, 100.0, 102.0, reason));
    identity(0, 3) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(manager.OnRelocPose(identity, 100.0, 100.0, reason));
}
TEST_F(ManagerTest, RequiresServiceConfirmationAndFreshTracking) {
    Accept();
    Manager::StatusInfo status;
    status.level = Level::kTracking; status.scan_stamp = 100.2;
    EXPECT_FALSE(manager.OnStatus(status, 100.2));
    SendAndAcknowledge();
    manager.ConfirmStartTracking(100.3);
    EXPECT_FALSE(manager.OnStatus(status, 100.3));
    status.level = Level::kDegraded; status.scan_stamp = 100.4;
    EXPECT_FALSE(manager.OnStatus(status, 100.4));
    status.level = Level::kTracking;
    EXPECT_TRUE(manager.OnStatus(status, 100.4));
    EXPECT_TRUE(manager.StartupCompleted());
}
TEST_F(ManagerTest, RecoveringNeverRetriesInitializationFromCore) {
    Accept();
    EXPECT_FALSE(manager.Tick(1000.0).request_reloc);
    EXPECT_EQ(manager.GetState(), State::kRecovering);
}
TEST_F(ManagerTest, CompletedStartupDoesNotRequestRelocalizationAgain) {
    Complete();
    Manager::StatusInfo status; status.level = Level::kLost;
    EXPECT_TRUE(manager.OnStatus(status, 101.0));
    EXPECT_FALSE(manager.Tick(1000.0).request_reloc);
    std::string reason;
    EXPECT_FALSE(manager.OnRelocPose(identity, 101.0, 101.0, reason));
    status.level = Level::kTracking; status.scan_stamp = 101.1;
    EXPECT_TRUE(manager.OnStatus(status, 101.1));
    EXPECT_EQ(manager.GetState(), State::kNormalTracking);
}
TEST_F(ManagerTest, TransientTfWaitAndDegradedDoNotTriggerRecovery) {
    Complete();
    Manager::StatusInfo status; status.level = Level::kInit; status.waiting_for_tf = true;
    EXPECT_FALSE(manager.OnStatus(status, 101.0));
    status.level = Level::kDegraded;
    EXPECT_FALSE(manager.OnStatus(status, 101.1));
    EXPECT_EQ(manager.GetState(), State::kNormalTracking);
}
TEST_F(ManagerTest, TransportsPoseWithLocalMotionAndRejectsUnsyncedTime) {
    Request();
    Eigen::Matrix4d pose = identity; pose(0, 3) = 20;
    std::string reason;
    ASSERT_TRUE(manager.OnRelocPose(pose, 100.0, 100.0, reason));
    Eigen::Matrix4d odom = identity; odom(0, 3) = 1;
    manager.PushOdom(100.1, odom);
    Eigen::Matrix4d predicted;
    ASSERT_TRUE(manager.PredictAcceptedPose(100.1, predicted));
    EXPECT_NEAR(predicted(0, 3), 21.0, 1e-9);
    EXPECT_FALSE(manager.PredictAcceptedPose(99.0, predicted));
    EXPECT_FALSE(manager.PredictAcceptedPose(102.0, predicted));
}
TEST_F(ManagerTest, InitialSearchRetainsRetryPolicy) {
    Request();
    EXPECT_FALSE(manager.Tick(101.0).request_reloc);
    EXPECT_TRUE(manager.Tick(111.0).request_reloc);
}

TEST_F(ManagerTest, HeartbeatTimeoutOnlyAfterInitializationAndRosTimeAdvance) {
    Request();
    EXPECT_FALSE(manager.CheckHealth(110.0, 3.0));
    std::string reason;
    ASSERT_TRUE(manager.OnRelocPose(identity, 100.0, 100.0, reason));
    SendAndAcknowledge();
    manager.ConfirmStartTracking(100.1);
    Manager::StatusInfo status; status.level = Level::kTracking; status.scan_stamp = 100.2;
    manager.OnStatus(status, 100.2);
    manager.ConsumeStateChanged();
    for (int i = 0; i < 10; ++i) EXPECT_FALSE(manager.CheckHealth(100.2, 3.0));
    EXPECT_FALSE(manager.CheckHealth(103.2, 3.0));
    EXPECT_TRUE(manager.CheckHealth(103.3, 3.0));
    EXPECT_TRUE(manager.HeartbeatStale());
    EXPECT_EQ(manager.GetState(), State::kLost);
    EXPECT_TRUE(manager.ConsumeStateChanged());
    EXPECT_FALSE(manager.CheckHealth(104.0, 3.0));
    EXPECT_FALSE(manager.Tick(104.0).request_reloc);
}

TEST_F(ManagerTest, HeartbeatReturnNotifiesWithoutFalseTrackingRecovery) {
    Complete();
    ASSERT_TRUE(manager.CheckHealth(104.0, 3.0));
    manager.ConsumeStateChanged();
    Manager::StatusInfo status;
    EXPECT_FALSE(manager.OnStatus(status, 104.1)); // 未知状态不能刷新心跳。
    EXPECT_TRUE(manager.HeartbeatStale());
    EXPECT_FALSE(manager.ConsumeStateChanged());
    status.level = Level::kInit; status.waiting_for_tf = true;
    EXPECT_FALSE(manager.OnStatus(status, 104.2));
    EXPECT_FALSE(manager.HeartbeatStale());
    EXPECT_EQ(manager.GetState(), State::kLost);
    EXPECT_TRUE(manager.ConsumeStateChanged());
    status.level = Level::kDegraded;
    EXPECT_FALSE(manager.OnStatus(status, 104.3));
    EXPECT_EQ(manager.GetState(), State::kLost);
    status.level = Level::kTracking; status.scan_stamp = 104.4;
    EXPECT_TRUE(manager.OnStatus(status, 104.4));
    EXPECT_EQ(manager.GetState(), State::kNormalTracking);
}

TEST_F(ManagerTest, TimeoutNotifiesWhenAlreadyLost) {
    Complete();
    Manager::StatusInfo status; status.level = Level::kLost; status.fail_count = 8;
    manager.OnStatus(status, 101.0);
    manager.ConsumeStateChanged();
    EXPECT_TRUE(manager.CheckHealth(105.0, 3.0));
    EXPECT_TRUE(manager.ConsumeStateChanged());
    EXPECT_EQ(manager.LastStatus().fail_count, 8);
}

TEST_F(ManagerTest, HandoffRejectsOutOfOrderAndLateAcknowledgements) {
    EXPECT_FALSE(manager.ConfirmSetPose());
    EXPECT_FALSE(manager.ConfirmStartTracking(100.0));
    Accept();
    EXPECT_FALSE(manager.MarkStartTrackingSent());
    ASSERT_TRUE(manager.MarkSetPoseSent());
    EXPECT_FALSE(manager.MarkSetPoseSent());
    ASSERT_TRUE(manager.ConfirmSetPose());
    EXPECT_FALSE(manager.ConfirmSetPose());
    ASSERT_TRUE(manager.MarkStartTrackingSent());
    ASSERT_TRUE(manager.RejectStartTracking());
    EXPECT_EQ(manager.Stage(), Manager::InitializationStage::kWaitingForGlobal);
    ASSERT_TRUE(manager.MarkStartTrackingSent());
    manager.FailInitialization("service timeout");
    EXPECT_FALSE(manager.ConfirmStartTracking(101.0));
    EXPECT_FALSE(manager.RejectStartTracking());
    EXPECT_FALSE(manager.ConfirmSetPose());
    Manager::StatusInfo status; status.level = Level::kTracking; status.scan_stamp = 102.0;
    EXPECT_FALSE(manager.OnStatus(status, 102.0));
    EXPECT_TRUE(manager.InitializationFailed());
    EXPECT_FALSE(manager.StartupCompleted());
    EXPECT_FALSE(manager.Tick(1000.0).request_reloc);
    EXPECT_TRUE(manager.ConsumeStateChanged());
    manager.FailInitialization("late error");
    EXPECT_FALSE(manager.ConsumeStateChanged());
    EXPECT_EQ(manager.StateReason(), "service timeout");
}

TEST_F(ManagerTest, OldTaskResultDoesNotScheduleFastRetry) {
    Request(); std::string reason;
    EXPECT_FALSE(manager.OnRelocPose(identity, 99., 100.1, reason));
    EXPECT_NE(reason.find("pre-request"), std::string::npos);
    EXPECT_FALSE(manager.Tick(103.).request_reloc);
    manager.PushOdom(103., identity);
    EXPECT_TRUE(manager.OnRelocPose(identity, 103., 103., reason));
}
TEST_F(ManagerTest, TimeRejectionRetriesOnceWithoutDuplicatePostponement) {
    Request(); std::string reason;
    EXPECT_FALSE(manager.OnRelocPose(identity, 100., 102., reason));
    EXPECT_NE(reason.find("stale"), std::string::npos);
    EXPECT_FALSE(manager.OnRelocPose(identity, 100., 103., reason));
    EXPECT_FALSE(manager.Tick(103.9).request_reloc);
    EXPECT_TRUE(manager.Tick(104.).request_reloc);
    EXPECT_FALSE(manager.Tick(104.1).request_reloc);
    manager.MarkRelocRequested();
    manager.PushOdom(104.2, identity);
    EXPECT_TRUE(manager.OnRelocPose(identity, 104.2, 104.3, reason));
    EXPECT_FALSE(manager.Tick(110.).request_reloc);
}
TEST_F(ManagerTest, FastRetriesAreBoundedAndPeriodicRetryRemains) {
    Request(); std::string reason;
    for (int i=0; i<3; ++i) {
        double request = 100. + 4*i;
        EXPECT_FALSE(manager.OnRelocPose(identity, request, request+2., reason));
        EXPECT_TRUE(manager.Tick(request+4.).request_reloc);
        manager.MarkRelocRequested();
    }
    EXPECT_FALSE(manager.OnRelocPose(identity, 112., 114., reason));
    EXPECT_NE(reason.find("exhausted"), std::string::npos);
    EXPECT_FALSE(manager.Tick(116.).request_reloc);
    EXPECT_TRUE(manager.Tick(122.).request_reloc);
}
