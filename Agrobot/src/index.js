import React from 'react';
import Routes from './routes';
import {StatusBar} from 'react-native';
import {DataContextProvider} from './context/dataContext.js';
import {CommunicationProvider} from './context/communication';

export default App = () => (
  <CommunicationProvider>
    <DataContextProvider>
      <StatusBar
        barStyle="dark-content"
        hidden={false}
        backgroundColor="#fff"
      />
      <Routes />
    </DataContextProvider>
  </CommunicationProvider>
);
