import React from 'react';
import { withStyles } from '@material-ui/core/styles';

import RosStore from '../stores/RosStore';
import RosActions from '../actions/RosActions';

const styles = theme => ({

});

class CanopenMotor extends React.Component {
    constructor(...args) {
        super(...args);

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

    }

    render() {
        return (
            <h1>{this.props.canopenNode}</h1>
        );
    }
}

export default withStyles(styles)(CanopenMotor);