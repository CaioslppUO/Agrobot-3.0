import React from 'react';

import {createDrawerNavigator} from '@react-navigation/drawer';
import {NavigationContainer} from '@react-navigation/native';

import Control from './pages/Control';
import Communication from './pages/Communication';
SetAutoParameters;
import SetAutoParameters from './pages/SetAutoParameters';

const Drawer = createDrawerNavigator();

export default App = () => (
  <NavigationContainer>
    <Drawer.Navigator
      initialRouteName="Control"
      drawerStyle={{backgroundColor: '#FFFFFF'}}
      drawerContentOptions={{
        activeBackgroundColor: '#efefef',
        labelStyle: {color: '#000', fontWeight: 'bold'},
        itemStyle: {
          borderBottomColor: '#000',
          borderBottomWidth: 2,
        },
      }}>
      <Drawer.Screen name="Controle" component={Control} />
      <Drawer.Screen name="Comunicação" component={Communication} />
      <Drawer.Screen
        name="Configuração parâmetros"
        component={SetAutoParameters}
      />
    </Drawer.Navigator>
  </NavigationContainer>
);
