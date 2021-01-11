import {View, TextInput, Text} from 'react-native';
import React from 'react';

import {CommunicationContext} from '../../context/communication';

import Button from '../../component/Button';
import Style from './style';

export default function Communication({navigation}) {
  const {updateAddress, updatePort, port, address} = React.useContext(
    CommunicationContext,
  );
  return (
    <>
      <View style={Style.container}>
        <Text style={Style.title}>Comunicação</Text>
        <View style={Style.containerInput}>
          <TextInput
            style={Style.inputText}
            placeholder={'IP: ' + address}
            onEndEditing={(text) => {
              if (text.nativeEvent.text != '')
                updateAddress(text.nativeEvent.text);
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'Porta: ' + port}
            onEndEditing={(text) => {
              if (text.nativeEvent.text != '')
                updatePort(text.nativeEvent.text);
            }}
            keyboardType="numeric"
          />
        </View>
        <Button
          style={Style.buttonSave}
          styleText={Style.textButtonSave}
          onPress={() => {
            navigation.navigate('Controle');
          }}>
          Salvar
        </Button>
      </View>
    </>
  );
}
