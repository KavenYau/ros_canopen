import React from 'react';
import { withStyles } from '@material-ui/core/styles';

import InputLabel from '@material-ui/core/InputLabel';
import MenuItem from '@material-ui/core/MenuItem';
import FormControl from '@material-ui/core/FormControl';
import Select from '@material-ui/core/Select';

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
            motorState: RosStore.getState().getIn(['canopenMotorStates', canopenNode])
        };
    }

    componentDidMount() {
        RosStore.on('change', this.storeChange);
        RosActions.connectCanopenMotor(this.props.canopenNode);
    }

    componentWillUnmount() {
        RosStore.removeListener('change', this.storeChange);
        RosActions.disconnectCanopenMotor(this.props.canopenNode);
    }

    storeChange = () => {
        const { canopenNode } = this.props;
        this.setState({
            motorState: RosStore.getState().getIn(['canopenMotorStates', canopenNode])
        });
    }

    handleModeChange = event => {
    }

    handle402StateChange = event => {
        RosActions.callCanopenMotorSwitch402State(event.target.value);
    }

    render() {
        const { motorState } = this.state;
        const { classes } = this.props;

        let activeOperationMode = 'No Mode';
        let active402State = 'Start';

        if (typeof motorState !== 'undefined')
        {
            const {operationMode, state402 } = motorState.toJS();
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
        operationModes.forEach( operationMode => {
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
        states402.forEach( state402 => {
            state402MenuItems.push(
                <MenuItem
                    key={state402}
                    value={state402}
                >
                    {state402}
                </MenuItem>
            );
        });

        return (
            <div>
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
                <FormControl className={classes.formControl}>
                    <InputLabel id='canopen-motor-state-label'>
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
            </div>
        );
    }
}

export default withStyles(styles)(CanopenMotor);