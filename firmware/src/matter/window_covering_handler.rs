use alloc::sync::Arc;
use core::cell::Cell;

use rs_matter_embassy::matter::dm::{Cluster, Dataver};
use rs_matter_embassy::matter::error::ErrorCode;
use rs_matter_embassy::matter::reexport::bitflags;
use rs_matter_embassy::matter::{import, with};

use window_covering::{
    ConfigStatus, EndProductType, GoToLiftPercentageRequest, GoToLiftValueRequest,
    GoToTiltPercentageRequest, GoToTiltValueRequest, Mode,
    OperationalStatus as MatterOperationalStatus, SafetyStatus, Type,
};

use crate::EmbassyMutex;
use crate::matter::Position;
use crate::movement::StepperController;
use crate::storage::Storage;

import!(WindowCovering);

const POSITION_100THS_MAX: u16 = 10_000; // 100% in 100ths

// This is a workaround for
// https://github.com/project-chip/rs-matter/issues/282
bitflags::bitflags! {
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    struct OperationalStatus: u8 {
        const GLOBAL_OPENING    = 0x01; // 0b0000_0001
        const GLOBAL_CLOSING    = 0x02; // 0b0000_0010

        const LIFT_OPENING      = 0x04; // 0b0000_0100
        const LIFT_CLOSING      = 0x08; // 0b0000_1000

        const TILT_OPENING      = 0x10; // 0b0001_0000
        const TILT_CLOSING      = 0x20; // 0b0010_0000
    }
}

impl OperationalStatus {
    const OPENING: Self = Self::GLOBAL_OPENING.union(Self::LIFT_OPENING);
    const CLOSING: Self = Self::GLOBAL_CLOSING.union(Self::LIFT_CLOSING);
}

impl From<OperationalStatus> for MatterOperationalStatus {
    fn from(status: OperationalStatus) -> Self {
        Self::from_bits_retain(status.bits())
    }
}

pub struct WindowCoveringHandler {
    dataver: Dataver,
    mode: Cell<Mode>,
    controller: Arc<EmbassyMutex<StepperController<'static>>>,
    storage: Storage<'static>,
}

impl WindowCoveringHandler {
    /// Create a new instance of the handler
    pub fn new(
        dataver: Dataver,
        controller: Arc<EmbassyMutex<StepperController<'static>>>,
        storage: Storage<'static>,
    ) -> Self {
        Self {
            dataver,
            mode: Cell::new(Mode::empty()),
            controller,
            storage,
        }
    }

    /// Adapt the handler instance to the generic `rs-matter` `AsyncHandler` trait
    pub const fn adapt(self) -> window_covering::HandlerAsyncAdaptor<Self> {
        window_covering::HandlerAsyncAdaptor(self)
    }

    pub async fn number_of_actuations_lift(&self) -> u16 {
        self.storage
            .number_of_actuations()
            .await
            .unwrap()
            .unwrap_or(0)
    }

    pub async fn increment_number_of_actuations_lift(&self) {
        let actuations = self.number_of_actuations_lift().await;
        self.storage
            .set_number_of_actuations(actuations.saturating_add(1))
            .await
            .unwrap();
    }

    pub async fn position(&self) -> Option<Position> {
        Some(self.controller.lock().await.current_position())
    }

    async fn target_position(&self) -> Option<Position> {
        self.controller.lock().await.target_position().await
    }

    pub async fn set_target_position(
        &self,
        position: Position,
    ) -> Result<(), rs_matter::error::Error> {
        log::info!(
            "Updating target position from {:?} to Some({position:?})",
            self.target_position().await
        );
        self.increment_number_of_actuations_lift().await;

        self.controller
            .lock()
            .await
            .update_target_position(Some(position))
            .await
            .map_err(|err| {
                log::error!("An error occurred while setting target position: {err}");
                rs_matter::error::Error::new(ErrorCode::StdIoError)
            })?;

        Ok(())
    }
}

pub trait FloatExt {
    fn round(self) -> Self;

    fn floor(self) -> Self;
}

impl FloatExt for f64 {
    fn round(self) -> Self {
        (self + 0.5).floor()
    }

    fn floor(self) -> Self {
        self - (self % 1.0)
    }
}

impl window_covering::ClusterAsyncHandler for WindowCoveringHandler {
    /// The metadata cluster definition corresponding to the handler
    const CLUSTER: Cluster<'static> = window_covering::FULL_CLUSTER
        // Indicate that the code supports Matter 1.4
        .with_revision(5)
        .with_features(
            // Support lift control and behavior for lifting window coverings
            window_covering::Feature::LIFT
                // Support position aware lift control (it knows where the limits are)
                .union(window_covering::Feature::POSITION_AWARE_LIFT)
                .bits(),
        )
        // For the features LIFT (LF) and POSITION_AWARE_LIFT (PA_LF),
        // the following attributes should be implemented:
        .with_attrs(with!(
            window_covering::AttributeId::Type
                //| window_covering::AttributeId::PhysicalClosedLimitLift
                | window_covering::AttributeId::NumberOfActuationsLift
                | window_covering::AttributeId::ConfigStatus
                | window_covering::AttributeId::CurrentPositionLiftPercentage
                | window_covering::AttributeId::OperationalStatus
                | window_covering::AttributeId::TargetPositionLiftPercent100ths
                | window_covering::AttributeId::EndProductType
                | window_covering::AttributeId::CurrentPositionLiftPercent100ths
                //| window_covering::AttributeId::VelocityLift
                //| window_covering::AttributeId::AccelerationTimeLift
                //| window_covering::AttributeId::DecelerationTimeLift
                | window_covering::AttributeId::Mode
                //| window_covering::AttributeId::IntermediateSetpointsLift
                | window_covering::AttributeId::SafetyStatus
                | window_covering::AttributeId::GeneratedCommandList
                | window_covering::AttributeId::AcceptedCommandList
                | window_covering::AttributeId::AttributeList
                | window_covering::AttributeId::FeatureMap
                | window_covering::AttributeId::ClusterRevision
        ))
        .with_cmds(with!(
            window_covering::CommandId::UpOrOpen
                | window_covering::CommandId::DownOrClose
                | window_covering::CommandId::StopMotion
                | window_covering::CommandId::GoToLiftValue
                | window_covering::CommandId::GoToLiftPercentage
        ));

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    async fn r#type(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<Type, rs_matter::error::Error> {
        Ok(Type::RollerShade)
    }

    /// This attribute SHALL indicate the total number of lift/slide actuations applied to the
    /// window covering since the device was installed
    async fn number_of_actuations_lift(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u16, rs_matter::error::Error> {
        Ok(self.number_of_actuations_lift().await)
    }

    /// This attribute specifies the configuration and status information of the window covering.
    ///
    /// To change settings, devices SHALL write to the Mode attribute. The behavior causing the setting or
    /// clearing of each bit is vendor specific
    async fn config_status(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<ConfigStatus, rs_matter::error::Error> {
        let mut result = ConfigStatus::empty();

        // The SafetyStatus & Mode attributes might affect this bit state.
        result |= ConfigStatus::OPERATIONAL;
        result |= ConfigStatus::LIFT_POSITION_AWARE;

        // This bit identifies if the directions of the lift/slide movements have
        // been reversed in order for commands (e.g: Open, Close, GoTos) to match the physical installation conditions
        //
        // This bit can be adjusted by setting the MotorDirectionReversed bit in the Mode attribute.
        //result |= ConfigStatus::LIFT_MOVEMENT_REVERSED;
        // This bit is ignored if the device does not support the PositionAwareLift feature (PA_LF).
        //result |= ConfigStatus::LIFT_ENCODER_CONTROLLED;
        // This bit is ignored if the device does not support the PositionAwareTilt feature (PA_TL).
        //result |= ConfigStatus::TILT_ENCODER_CONTROLLED;

        /*if self.controller.is_reversed() {
            result |= ConfigStatus::LIFT_MOVEMENT_REVERSED;
        }*/

        Ok(result)
    }

    /// The current position as a percentage from 0 to 100, with a default step of 1%.
    ///
    /// This attribute should be equal to the [`WindowCoveringHandler::current_position_lift_percent_100_ths`] divided by 100.
    async fn current_position_lift_percentage(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<u8>, rs_matter::error::Error> {
        Ok(
            Option::from(self.current_position_lift_percent_100_ths(ctx).await?)
                .map(|value: u16| (value as f64 / 100.0).round() as u8)
                .into(),
        )
    }

    async fn current_position_lift_percent_100_ths(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<u16>, rs_matter::error::Error> {
        let result = self
            .position()
            .await
            .map(|pos| pos.map_to(POSITION_100THS_MAX as f64) as u16);

        log::info!("Requested current position lift percent 100ths: {result:?}");

        Ok(result.into())
    }

    async fn target_position_lift_percent_100_ths(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<rs_matter::tlv::Nullable<u16>, rs_matter::error::Error> {
        let result = self
            .target_position()
            .await
            .map(|pos| pos.map_to(POSITION_100THS_MAX as f64) as u16);

        log::info!("Requested target position lift percent 100ths: {result:?}");

        Ok(result.into())
    }

    /// This attribute SHALL indicate the currently ongoing operations and applies to all type of devices.
    async fn operational_status(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<MatterOperationalStatus, rs_matter::error::Error> {
        let mut status = OperationalStatus::empty();

        match (self.position().await, self.target_position().await) {
            (Some(current), Some(target)) if current != target => {
                if current.is_opening(target) {
                    status |= OperationalStatus::OPENING;
                } else {
                    status |= OperationalStatus::CLOSING;
                }
            }
            (_, None) | (Some(_), Some(_)) => {
                // When no target position is set, or the current position is equal to the target position,
                // there is no movement -> do not set any status bits
            }
            (None, Some(target)) => {
                // Position is unknown, but there is a target position?
                log::error!("Current position is None, but target position is Some({target:?})");

                if target.is_opening(Position::new(0.5)) {
                    status |= OperationalStatus::OPENING;
                } else {
                    status |= OperationalStatus::CLOSING;
                }
            }
        }

        log::info!("Responding with operational status: {status:?}");

        Ok(status.into())
    }

    /// Provides a bit more details about the end product type.
    async fn end_product_type(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<EndProductType, rs_matter::error::Error> {
        Ok(EndProductType::RollerShade)
    }

    /// The Mode attribute allows configuration of the window covering, such as:
    /// - reversing the motor direction,
    /// - placing the window covering into calibration mode,
    /// - placing the motor into maintenance mode,
    /// - disabling the network,
    /// - and disabling status LEDs.
    ///
    /// In the case a device does not support or implement a specific mode, e.g. the device has a specific
    /// installation method and reversal is not relevant or the device does not include a maintenance
    /// mode, any write interaction to the Mode attribute, with an unsupported mode bit or any out of
    /// bounds bits set, must be ignored and a response containing the status of CONSTRAINT_ERROR will
    /// be returned.
    async fn mode(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<Mode, rs_matter::error::Error> {
        log::info!("Asked for mode, returning: {:?}", self.mode.get());
        Ok(self.mode.get())
    }

    /// The SafetyStatus attribute reflects the state of the safety sensors and the common issues preventing
    /// movements. By default for nominal operation all flags are cleared (0). A device might support none,
    /// one or several bit flags from this attribute (all optional).
    async fn safety_status(
        &self,
        _ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<SafetyStatus, rs_matter::error::Error> {
        log::info!("Asked for safety status, returning empty SafetyStatus");
        Ok(SafetyStatus::empty())
    }

    async fn set_mode(
        &self,
        _ctx: impl rs_matter::dm::WriteContext,
        value: Mode,
    ) -> Result<(), rs_matter::error::Error> {
        log::info!("Setting mode: {value:?}");
        self.mode.set(value);
        Ok(())
    }

    async fn handle_up_or_open(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
    ) -> Result<(), rs_matter::error::Error> {
        log::info!("Called handle_up_or_open");

        self.set_target_position(Position::open()).await?;

        Ok(())
    }

    async fn handle_down_or_close(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
    ) -> Result<(), rs_matter::error::Error> {
        log::info!("Called handle_down_or_close");

        self.set_target_position(Position::closed()).await?;

        Ok(())
    }

    async fn handle_stop_motion(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
    ) -> Result<(), rs_matter::error::Error> {
        log::info!("Handling Stop Motion command");

        // This command will stop the current motion of the window covering, if any.
        self.controller.lock().await.stop().await.map_err(|err| {
            log::error!("An error occurred while stopping motion: {err}");
            rs_matter::error::Error::new(ErrorCode::StdIoError)
        })?;

        Ok(())
    }

    async fn handle_go_to_lift_percentage(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        request: GoToLiftPercentageRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        let target = Position::new(
            request.lift_percent_100_ths_value()? as f64 / POSITION_100THS_MAX as f64,
        );
        log::info!(
            "Handling Go To Lift Percentage command with percentage: {}_100ths, target position: {target:?}",
            request.lift_percent_100_ths_value()?
        );

        self.set_target_position(target).await
    }

    // The below functions have to be implemented, but are not supported by this device,
    // -> they return the appropriate error code.

    async fn handle_go_to_lift_value(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GoToLiftValueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(ErrorCode::InvalidAction.into())
    }

    async fn handle_go_to_tilt_percentage(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GoToTiltPercentageRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(ErrorCode::InvalidAction.into())
    }

    async fn handle_go_to_tilt_value(
        &self,
        _ctx: impl rs_matter::dm::InvokeContext,
        _request: GoToTiltValueRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        Err(ErrorCode::InvalidAction.into())
    }
}
