import React from 'react';
import { withStyles } from '@material-ui/core/styles';

import InputLabel from '@material-ui/core/InputLabel';
import MenuItem from '@material-ui/core/MenuItem';
import FormControl from '@material-ui/core/FormControl';
import Select from '@material-ui/core/Select';
import Slider from '@material-ui/core/Slider';
import Typography from '@material-ui/core/Typography';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import TextField from '@material-ui/core/TextField';
import Grid from '@material-ui/core/Grid';

import RosStore from '../stores/RosStore';
import RosActions from '../actions/RosActions';
// import { secondaryListItems } from './ListItems';

const styles = theme => ({
    formControl: {
        margin: theme.spacing(1),
        minWidth: 120
    },
    selectEmpty: {
        marginTop: theme.spacing(2)
    }
});

class CanopenMotor extends React.Component {
    constructor(...args) {
        super(...args);

        const { canopenNode } = this.props;

        this.state = {
            motorState: RosStore.getState().getIn(['canopenMotorStates', canopenNode]),
            targetVelocity: 0.0,
            profileAcceleration: 200,
            profileDeceleration: 200,
            targetVelocityRange: '100',
            profileAccelerationRange: '1000',
            profileDecelerationRange: '1000'
        };
    }

    componentDidMount() {
        RosStore.on('change', this.storeChange);
        RosActions.connectCanopenMotor(this.props.canopenNode);
        this.manualCommandInterval = setInterval(
            this.sendManualMotorCommands,
            1000
        );
    }

    componentWillUnmount() {
        RosStore.removeListener('change', this.storeChange);
        RosActions.disconnectCanopenMotor(this.props.canopenNode);
        clearInterval(this.manualCommandInterval);
    }

    sendManualMotorCommands = () => {
        RosActions.publishProfiledVelocity(
            this.state.targetVelocity,
            this.state.profileAcceleration,
            this.state.profileDeceleration);
    }

    storeChange = () => {
        const { canopenNode } = this.props;
        this.setState({
            motorState: RosStore.getState().getIn(['canopenMotorStates', canopenNode])
        });
    }

    handleModeChange = event => {
        RosActions.callCanopenMotorSwitchOperationMode(event.target.value);
    }

    handle402StateChange = event => {
        RosActions.callCanopenMotorSwitch402State(event.target.value);
    }

    handleTargetVelocityChange = (event, newValue) => {
        this.setState({
            targetVelocity: newValue
        });
    }

    handleProfileAccelerationChange = (event, newValue) => {
        this.setState({
            profileAcceleration: newValue
        });
    }

    handleProfileDecelerationChange = (event, newValue) => {
        this.setState({
            profileDeceleration: newValue
        });
    }

    handleTargetVelocityRangeChange = event => {
        this.setState({
            targetVelocityRange: event.target.value
        });
    }

    handleProfileAccelerationRangeChange = event => {
        this.setState({
            profileAccelerationRange: event.target.value
        });
    }

    handleProfileDecelerationRangeChange = event => {
        this.setState({
            profileDecelerationRange: event.target.value
        });
    }

    render() {
        const { 
            motorState, 
            targetVelocity,
            profileAcceleration, 
            profileDeceleration,
            targetVelocityRange,
            profileAccelerationRange,
            profileDecelerationRange
        } = this.state;

        const { classes } = this.props;

        let activeOperationMode = 'No Mode';
        let active402State = 'Start';

        if (typeof motorState !== 'undefined') {
            const { operationMode, state402 } = motorState.toJS();
            activeOperationMode = operationMode;
            active402State = state402;
        }

        const operationModes = [
            'No Mode',
            'Profiled Position',
            // 'Velocity',
            'Profiled Velocity',
            'Profiled Torque',
            // 'Reserved',
            'Homing',
            // 'Interpolated Position',
            // 'Cyclic Synchronous Position',
            // 'Cyclic Synchronous Velocity',
            // 'Cyclic Synchronous Torque'
        ];

        const operationModeMenyItems = [];
        operationModes.forEach(operationMode => {
            operationModeMenyItems.push(
                <MenuItem
                    key={operationMode}
                    value={operationMode}
                >
                    {operationMode}
                </MenuItem>
            );
        });

        const states402 = [
            'Start',
            'Not Ready To Switch On',
            'Switch On Disabled',
            'Ready To Switch On',
            'Switched On',
            'Operation Enable',
            'Quick Stop Active',
            'Fault Reaction Active',
            'Fault'
        ];

        const state402MenuItems = [];
        states402.forEach(state402 => {
            state402MenuItems.push(
                <MenuItem
                    key={state402}
                    value={state402}
                >
                    {state402}
                </MenuItem>
            );
        });

        const profiledVelocityControls = (
            <React.Fragment>
                <Typography component="div">
                    <Box fontSize="h6.fontSize" m={3} textAlign="center">
                        Profiled Velocity Controls
                    </Box>
                </Typography>
                <Typography id="profile-velocity-slider">
                    Target Velocity
                </Typography>
                <Slider
                    value={targetVelocity}
                    defaultValue={0.00000005}
                    aria-labelledby="profile-velocity-slider"
                    step={0.1}
                    min={-parseInt(targetVelocityRange)}
                    max={parseInt(targetVelocityRange)}
                    valueLabelDisplay="auto"
                    onChange={this.handleTargetVelocityChange}
                />
                <Grid container justify="space-around">
                    <Grid item>
                        <Button
                            variant="contained"
                            color="primary"
                            onClick={() => { this.setState({ targetVelocity: 0.0 }) }}
                        >
                            Set To Zero
                        </Button>
                    </Grid>
                    <Grid item>
                        <TextField 
                            id="target-velocity-range" 
                            label="Velocity Range" 
                            variant="filled" 
                            type="number"
                            value={targetVelocityRange}
                            onChange={this.handleTargetVelocityRangeChange}
                        />
                    </Grid>
                </Grid>
                <Typography id="profile-acceleration-slider">
                    Profile Acceleration
                </Typography>
                <Slider
                    value={profileAcceleration}
                    defaultValue={0.00000005}
                    aria-labelledby="profile-acceleration-slider"
                    step={1}
                    min={0}
                    max={parseInt(profileAccelerationRange)}
                    valueLabelDisplay="auto"
                    onChange={this.handleProfileAccelerationChange}
                />
                <Typography id="profile-deceleration-slider">
                    Profile Deceleration
                </Typography>
                <Slider
                    value={profileDeceleration}
                    defaultValue={0.00000005}
                    aria-labelledby="profile-deceleration-slider"
                    step={1}
                    min={0}
                    max={parseInt(profileDecelerationRange)}
                    valueLabelDisplay="auto"
                    onChange={this.handleProfileDecelerationChange}
                />
                <Grid container justify="space-between">
                    <Grid item>
                        <TextField 
                            id="profile-acceleration-range" 
                            label="Acceleration Range" 
                            variant="filled" 
                            type="number"
                            value={profileAccelerationRange}
                            onChange={this.handleProfileAccelerationRangeChange}
                        />
                    </Grid>
                    <Grid item>
                        <TextField 
                            id="profile-deceleration-range" 
                            label="Deleleration Range" 
                            variant="filled" 
                            type="number"
                            value={profileDecelerationRange}
                            onChange={this.handleProfileDecelerationRangeChange}
                        />
                    </Grid>
                </Grid>
            </React.Fragment>
        );

        let manualControls = (<div></div>);
        
        if (activeOperationMode === 'Profiled Velocity')
        {
            manualControls = profiledVelocityControls;
        }

        return (
            <div>
                <Grid container justify="space-around">
                    <Grid item>
                        <FormControl className={classes.formControl}>
                            <InputLabel id='canopen-motor-mode-label'>
                                Operation Mode
                            </InputLabel>
                            <Select
                                labelId="canopen-motor-mode-label"
                                id="canopen-motor-mode-select"
                                value={activeOperationMode}
                                onChange={this.handleModeChange}
                            >
                                {operationModeMenyItems}
                            </Select>
                        </FormControl>
                    </Grid>
                    <Grid item>
                        <FormControl className={classes.formControl}>
                            <InputLabel id="canopen-motor-state-label">
                                402 State
                            </InputLabel>
                            <Select
                                labelId="canopen-motor-state-label"
                                id="canopen-motor-state-select"
                                value={active402State}
                                onChange={this.handle402StateChange}
                            >
                                {state402MenuItems}
                            </Select>
                        </FormControl>
                    </Grid>
                </Grid>
                {manualControls}
            </div>
        );
    }
}

export default withStyles(styles)(CanopenMotor);