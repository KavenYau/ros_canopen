
/* eslint-disable no-script-url */
import React from 'react';
import { withStyles } from '@material-ui/core/styles';
import Grid from '@material-ui/core/Grid'

import CommonActions from '../actions/RosActions';
import RosStore from '../stores/RosStore';
import IconButton from '@material-ui/core/IconButton';
import RefreshIcon from '@material-ui/icons/Refresh';

import MaterialTable from 'material-table';

import Switch from '@material-ui/core/Switch';
import FormControlLabel from '@material-ui/core/FormControlLabel';

const styles = theme => ({
  formControl: {
    margin: theme.spacing(1),
    minWidth: 140,
  }
});

class Rosparams extends React.Component {
  constructor(...args) {
    super(...args);

    this.state = {
      rosParams: RosStore.getState().get('rosParams'),
      canopenObjectDictionaries: RosStore.getState().get('canopenObjectDictionaries'),
      cached: false
    };
  }

  componentDidMount() {
    RosStore.on('change', this.storeChange);
  }

  componentWillUnmount() {
    RosStore.removeListener('change', this.storeChange);
  }

  storeChange = () => {
    // Only update state if params of objectDictionary in store has changed
    // Running render while editing a Material Table row seems to break stuff
    if (this.state.rosParams !== RosStore.getState().get('rosParams') ||
        this.state.canopenObjectDictionaries !== RosStore.getState().get('canopenObjectDictionaries'))
    {
      this.setState({
        rosParams: RosStore.getState().get('rosParams'),
        canopenObjectDictionaries: RosStore.getState().get('canopenObjectDictionaries')
      });
    }
  }

  handleToDeviceChange = event => {
    this.setState({
      cached: event.target.checked
    });
  }

  render() {
    const { canopenNode } = this.props;

    let canopenDictionaryEntries = [];
    if (this.state.canopenObjectDictionaries.get(canopenNode))
    {
      canopenDictionaryEntries = this.state.canopenObjectDictionaries.get(canopenNode).toJS()
    }

    const data_type_names = [
      'Unknown',
      'Unknown',
      'Int8',
      'Int16',
      'Int32',
      'UInt8',
      'UInt16',
      'UInt32',
      'Real32',
      'VisibleString',
      'OctetString',
      'UnicodeString',
      'Unknow',
      'Unknown',
      'Unknown',
      'Domain',
      'Real64',
      'Unknow',
      'Unknown',
      'Unknow',
      'Unknown',
      'Int64',
      'Unknow',
      'Unknown',
      'Unknow',
      'Unknown',
      'Unknown',
      'UInt64',
    ]

    canopenDictionaryEntries.forEach( entry => {
      entry.data_type_name = data_type_names[entry.data_type]
    });

    return (
      <React.Fragment>
        <Grid container justify='center'>
        <Grid item>
        <FormControlLabel
          control={
            <Switch
              checked={this.state.cached}
              onChange={this.handleToDeviceChange}
            />
          }
          label='Cached'
        />
        </Grid>
        <Grid item>
          <IconButton
            onClick={() => CommonActions.refreshObjectDictionaries([canopenNode])}
          >
            <RefreshIcon color='primary'/>
          </IconButton>
        </Grid>
        </Grid>
        <MaterialTable
          columns={[
            { title: 'Index', field: 'index', editable: 'never'},
            { title: 'Parameter Name', field: 'parameter_name', editable: 'never'},
            { title: 'Data Type', field: 'data_type_name', editable: 'never'},
            { title: 'Value', field: 'value' },
          ]}
          data={canopenDictionaryEntries}
          title='Object Dictionary'
          editable={{
            onRowUpdate: (newData, oldData) => 
              new Promise(resolve => {
                CommonActions.updateCanopenObject(newData, oldData, canopenNode);
                resolve();
              })}}
          actions={[
            rowData => ({
              icon: 'refresh',
              tooltip: 'Read value',
              onClick: (event, rowData) => CommonActions.callCanopenGetObjectService(canopenNode, rowData, this.state.cached)
            })
          ]}
        />
      </React.Fragment>
    );
  }
}

export default withStyles(styles)(Rosparams);