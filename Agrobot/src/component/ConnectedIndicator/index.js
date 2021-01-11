import React from 'react';
import {View, StyleSheet, Text, Dimensions} from 'react-native';
const heightScreen = Dimensions.get('window').height;
export default function ConnectedIndicator({connected = false}) {
  return (
    <View style={Styles.Container}>
      <Text style={Styles.IndicatorText}>
        {connected ? 'Conectado' : 'Desconectado'}
      </Text>
      <View style={Styles.IndicatorIcon}>
        <View
          style={[
            Styles.Indicator,
            {backgroundColor: connected ? '#0f0' : '#f00'},
          ]}
        />
      </View>
    </View>
  );
}

const Styles = StyleSheet.create({
  Container: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  IndicatorText: {
    color: '#000',
    marginRight: 15,
    marginTop: 5,
    marginBottom: 2,
  },
  IndicatorIcon: {
    width: 15,
    height: 15,
    borderWidth: 1,
    borderColor: '#000',
    alignItems: 'center',
    justifyContent: 'center',
    marginTop: 5,
    marginRight: 15,
  },
  Indicator: {
    width: 10,
    height: 10,
  },
});
