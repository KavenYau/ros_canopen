import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import AppBar from '@material-ui/core/AppBar';
import Tabs from '@material-ui/core/Tabs';
import Tab from '@material-ui/core/Tab';
import Typography from '@material-ui/core/Typography';
import Box from '@material-ui/core/Box';

// import CommonActions from "@material-ui/core/colors/orange";
import RosStore from '../stores/RosStore';

import CanopenMotor from './CanopenMotor';
import CanopenObjectDictionary from './CanopenObjectDictionary';

const styles = theme => ({
    root: {
        flexGrow: 1,
        backgroundColor: theme.palette.background.paper,
    },
});

function TabPanel(props) {
  const { children, value, index, ...other } = props;

  return (
    <Typography
      component="div"
      role="tabpanel"
      hidden={value !== index}
      id={`simple-tabpanel-${index}`}
      aria-labelledby={`simple-tab-${index}`}
      {...other}
    >
      {value === index && <Box p={3}>{children}</Box>}
    </Typography>
  );
}


function a11yProps(index) {
    return {
        id: `simple-tab-${index}`,
        'aria-controls': `simple-tabpanel-${index}`,
    };
}

class CanopenNodes extends React.Component {
    constructor(...args) {
        super(...args);

        this.state = {
            value: 0,
            rosParams: RosStore.getState().get('rosParams')
        }


    }

    componentDidMount() {
        RosStore.on('change', this.storeChange);
    }

    componentWillUnmount() {
        RosStore.removeListener('change', this.storeChange);
    }

    storeChange = () => {
        if (this.state.rosParams !== RosStore.getState().get('rosParams'))
        {
            this.setState({
                rosParams: RosStore.getState().get('rosParams')
            });
        }
    }

    handleChange = (event, newValue) => {
        this.setState({
            value: newValue
        });
    }

    render() {
        const { classes } = this.props;
        const { value, rosParams } = this.state;

        let canopenNodes = [];
        rosParams.forEach(rosParam => {
            if(rosParam.get('name') === 'canopen_nodes') {
                canopenNodes = JSON.parse(rosParam.get('valueString'));
            }
        });

        const nodeTabs = [];
        const nodePanels = [];
        let index = 0;
        canopenNodes.forEach(canopenNode => {
            
            const panelContent = [];
            rosParams.forEach(rosParam => {
                if(rosParam.get('name') === canopenNode + '.profiles') {
                    panelContent.push(
                        <CanopenMotor 
                            key={canopenNode}
                            canopenNode={canopenNode}
                        />
                    );
                }
            });
            
            nodeTabs.push(
                <Tab key={canopenNode} label={canopenNode} {...a11yProps(index)} />
            );
            nodePanels.push(
                <TabPanel key={canopenNode} value={value} index={index}>
                    {panelContent}
                </TabPanel>
            );
            index++;
        });

        return (
            <div className={classes.root}>
                <AppBar position="static">
                    <Tabs value={value} onChange={this.handleChange} aria-label="simple tabs example">
                        {nodeTabs}
                    </Tabs>
                </AppBar>
                {nodePanels}
                <CanopenObjectDictionary canopenNode={canopenNodes[value]}/>
            </div>
        );
    }
}

export default withStyles(styles)(CanopenNodes);