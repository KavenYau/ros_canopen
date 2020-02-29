import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Title from './Title';
import List from '@material-ui/core/List';
import ListItem from '@material-ui/core/ListItem';
import { ListItemText, Snackbar, SnackbarContent} from '@material-ui/core';
import ListItemIcon from '@material-ui/core/ListItemIcon';
import InfoIcon from '@material-ui/icons/Info';
import WarningIcon from '@material-ui/icons/Warning';
import ErrorIcon from '@material-ui/icons/Error';
import CloseIcon from '@material-ui/icons/Close';
import ClearAll from '@material-ui/icons/ClearAll';
import Grid from '@material-ui/core/Grid';
import IconButton from "@material-ui/core/IconButton";
import orange from "@material-ui/core/colors/orange";

import RosStore from '../stores/RosStore';
import RosActions from '../actions/RosActions';

const styles = theme => ({
    ul: {
        padding: 0,
    },
    snackbarMessage: {
        display: 'flex',
        alignItems: 'center',
    },
    snackbarIcon: {
        opacity: 0.9,
        marginRight: theme.spacing(1),
    },
    error: {
        backgroundColor: theme.palette.error.dark,
    },
});


class Rosout extends React.Component {
    constructor(...args) {
        super(...args);

        this.state = {
            rosoutMsgs: RosStore.getState().get('rosoutMsgs'),
            snackbarOpen: false,
            snackbarMessage: ''
        }
    }

    componentDidMount() {
        RosStore.on('change', this.storeChange);
        RosActions.connectCanopenRosout('canopen_chain');
    }

    componentWillUnmount() {
        RosStore.removeListener('change', this.storeChange);
        RosActions.disconnectCanopenRosout('canopen_chain');
    }

    storeChange = () => {
        const rosoutMsgs = RosStore.getState().get('rosoutMsgs');
        this.setState({
            rosoutMsgs
        });
        
        const mostRecentMsg = rosoutMsgs.get(-1);
        if (typeof mostRecentMsg !== 'undefined' && 
            mostRecentMsg.get('level') === 40) { // If last message is error message
            if (this.state.snackbarMessage !== mostRecentMsg.get('msg')) {
                this.setState({
                    snackbarMessage: mostRecentMsg.get('msg'),
                    snackbarOpen: true
                });
            }
        }
    }

    handleClose = (event, reason) => {
        if (reason === 'clickaway') {
            return;
        }

        this.setState({
            snackbarOpen: false
        });
    };

    render() {
        const { classes } = this.props;
        const rosoutListItems = []

        this.state.rosoutMsgs.reverse().forEach(rosout_entry => {
            const stamp = rosout_entry.getIn(['stamp', 'sec']) + rosout_entry.getIn(['stamp', 'nanosec']);
            const level = rosout_entry.get('level');

            let icon;
            icon = <InfoIcon color='primary' />;
            if (level === 30) {
                icon = <WarningIcon style={{color: orange.A200}} />;
            } else if (level === 40) {
                icon = <ErrorIcon color='error' />;
            }

            rosoutListItems.push(
                <ListItem key={stamp}>
                    <ListItemIcon>
                        {icon}
                    </ListItemIcon>
                    <ListItemText>
                        {rosout_entry.get('msg')}
                    </ListItemText>
                </ListItem>
            );
        });

        return (
            <React.Fragment>
                <Snackbar
                    anchorOrigin={{
                        vertical: 'bottom',
                        horizontal: 'left',
                    }}
                    open={this.state.snackbarOpen}
                    autoHideDuration={6000}
                    onClose={this.handleClose}
                    
                >
                    <SnackbarContent 
                        className={classes.error}
                        message={
                            <span id="message-id" className={classes.snackbarMessage}>
                                <ErrorIcon className={classes.snackbarIcon}/>
                                {this.state.snackbarMessage}
                            </span>}
                        action={[
                            <IconButton
                                key="close"
                                aria-label="close"
                                color="inherit"
                                className={classes.close}
                                onClick={this.handleClose}
                            >
                                <CloseIcon />
                            </IconButton>,
                        ]}
                    />
                </Snackbar>

                <Grid container justify='space-between'>
                    <Grid item>
                        <Title>ROS Console</Title>
                    </Grid>
                    <Grid item>
                        <IconButton
                            onClick={() => RosActions.clearRosoutMessages()}
                        >
                            <ClearAll color='primary' />
                        </IconButton>
                    </Grid>
                </Grid>
                <List>
                    <ul className={classes.ul}>
                        {rosoutListItems}
                    </ul>
                </List>
            </React.Fragment>
        )
    }
}

export default withStyles(styles)(Rosout);