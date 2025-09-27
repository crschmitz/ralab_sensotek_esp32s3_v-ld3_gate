const char defaultParams[] PROGMEM = R"CFG(
% ***************************************************************
% IWRL_Tracking_MidBw_Sensitive: Chirp configuration and
% processing chain are designed to detect, localize, and track
% objects in indoor or outdoor environments with higher sensitivity.
% This config utilizes ~2GHz BW (~10cm range resolution) and
% enables all the available six antennas of the EVM.
% The maximum unambigious range of this config is ~10m.
% In this config, auto mode (major and minor) is enabled to track
% objects with any kind of motion, including fine movements.
% ***************************************************************
sensorStop 0
channelCfg 7 3 0
chirpComnCfg 10 0 0 128 4 28 0
chirpTimingCfg 6 32 0 100 57.5
frameCfg 2 0 500 32 150 0
antGeometryCfg 1 0 0 1 1 2 1 1 0 2 1 3 2.5 2.5
guiMonitor 2 3 0 0 0 1 0 0 1 1 1
sigProcChainCfg 16 32 2 0 4 4 0 .5
cfarCfg 2 8 4 3 0 9.0 0 0.5 0 1 1 1
aoaFovCfg -70 70 -40 40
rangeSelCfg 0.1 10.0
clutterRemoval 1
compRangeBiasAndRxChanPhase 0.0 1.00000 0.00000 -1.00000 0.00000 1.00000 0.00000 -1.00000 0.00000 1.00000 0.00000 -1.00000 0.00000
adcDataSource 0 adc_data_0001_CtestAdc6Ant.bin
adcLogging 0
lowPowerCfg 0
factoryCalibCfg 1 0 40 0 0x1ff000
boundaryBox -2 2 0 4 0 3
sensorPosition 0 0 2 0 0
staticBoundaryBox -2 2 0.5 4 0 3
gatingParam 3 2 2 2 4
stateParam 3 3 12 75 5 200
allocationParam 6 10 0.06 4 0.5 20
maxAcceleration 0.1 0.1 0.1
trackingCfg 1 2 100 5 61.4 191.8 100
presenceBoundaryBox -2 2 0.5 4 0 3
% baudRate 1250000
sensorStart 0 0 0 0
)CFG";
