/* eslint-disable no-script-url */
import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Button from '@material-ui/core/Button'
import Title from './Title';
import Grid from '@material-ui/core/Grid'

import { IconButton, Typography } from '@material-ui/core';
import RefreshIcon from '@material-ui/icons/Refresh';

import RosActions from '../actions/RosActions';
import RosStore from '../stores/RosStore';

const styles = theme => ({
  button: {
    margin: theme.spacing(1),
  },
});

class Lifecycle extends React.Component {
  constructor(...args) {
    super(...args);

    this.state = {
      availableLifecycleTransitions: RosStore.getState().get('availableLifecycleTransitions'),
      lifecycleState: RosStore.getState().get('lifecycleState')
    };
  }

  componentDidMount() {
    RosStore.on('change', this.storeChange);
    RosActions.connectLifecycle('canopen_chain');
  }

  componentWillUnmount() {
    RosStore.removeListener('change', this.storeChange);
    RosActions.disconnectLifecycle('canopen_chain');
  }

  storeChange = () => {
    this.setState({
      availableLifecycleTransitions: RosStore.getState().get('availableLifecycleTransitions'),
      lifecycleState: RosStore.getState().get('lifecycleState')
    })
  }

  render() {
    const { classes } = this.props;
    const { availableLifecycleTransitions } = this.state;

    const transitionButtons = [];

    availableLifecycleTransitions.forEach(transitionName => {
      transitionButtons.push(
        <Button
          variant="contained"
          color="primary"
          className={classes.button}
          key={transitionName}
          onClick={() => RosActions.callLifecycleChangeStateService(transitionName)}
        >
          {transitionName}
        </Button>
      )
    });

    return (
      <React.Fragment>
        <Grid container justify='space-between'>
        <Grid item>
          <Title>Lifecycle</Title>
        </Grid>
        <Grid item>
          <IconButton
            onClick={() => RosActions.callGetAvailableLifecycleTransitionsService()}
          >
            <RefreshIcon color='primary'/>
          </IconButton>
        </Grid>
        </Grid>
        <Typography>
          Current state: {this.state.lifecycleState}
        </Typography>
        <div>
          {transitionButtons}
        </div>
      </React.Fragment>
    );
  }
}

export default withStyles(styles)(Lifecycle);
