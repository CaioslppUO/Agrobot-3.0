import {ScrollView, View, TextInput, Text, Alert} from 'react-native';
import React from 'react';

import {DataContext} from '../../context/dataContext';

import Button from '../../component/Button';
import Style from './style';

export default function SetAutoParameters({navigation}) {
  const {
    updateModeData,
    autoModeData,
    autoMode,
    moduleRobot,
  } = React.useContext(DataContext);

  const limit = React.useRef(autoModeData.limit);
  const steer = React.useRef(autoModeData.steer);
  const speed = React.useRef(autoModeData.speed);
  const correctionsMovements = React.useRef(autoModeData.correctionsMovements);
  const correctionFactor = React.useRef(autoModeData.correctionFactor);
  const detectDistance = React.useRef(autoModeData.detectDistance);
  const moveTime = React.useRef(autoModeData.moveTime);
  const stopTime = React.useRef(autoModeData.stopTime);

  React.useEffect(() => {
    if (autoModeData) {
      limit.current = autoModeData.limit;
      steer.current = autoModeData.steer;
      speed.current = autoModeData.speed;
      correctionsMovements.current = autoModeData.correctionsMovements;
      correctionFactor.current = autoModeData.correctionFactor;
      detectDistance.current = autoModeData.detectDistance;
      moveTime.current = autoModeData.moveTime;
      stopTime.current = autoModeData.stopTime;
    }
  }, [autoModeData]);

  function handleSave() {
    const obj = {
      limit: limit.current,
      steer: steer.current,
      speed: speed.current,
      correctionsMovements: correctionsMovements.current,
      correctionFactor: correctionFactor.current,
      detectDistance: detectDistance.current,
      moveTime: moveTime.current,
      stopTime: stopTime.current,
      moduleRobot: moduleRobot,
      autoMode: autoMode,
    };
    updateModeData(obj);
    navigation.navigate('Controle');
  }

  return (
    <>
      <View style={Style.container}>
        <Text style={Style.title}>Parâmetros</Text>
        <View style={Style.containerInput}>
          <TextInput
            style={Style.inputText}
            placeholder={'Limite: ' + autoModeData.limit}
            onChangeText={(text) => {
              limit.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'Direção: ' + autoModeData.steer}
            onEndEditing={(text) => {
              steer.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'speed: ' + autoModeData.speed}
            onEndEditing={(text) => {
              speed.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={
              'N de movimentos de correção: ' +
              autoModeData.correctionsMovements
            }
            onEndEditing={(text) => {
              correctionsMovements.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'Fator de correção: ' + autoModeData.correctionFactor}
            onEndEditing={(text) => {
              correctionFactor.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'Distancia de colisão: ' + autoModeData.detectDistance}
            onEndEditing={(text) => {
              detectDistance.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'Andar por(seg): ' + autoModeData.moveTime}
            onEndEditing={(text) => {
              moveTime.current = text;
            }}
            keyboardType="numeric"
          />
          <TextInput
            style={Style.inputText}
            placeholder={'Parar por(Seg): ' + autoModeData.stopTime}
            onEndEditing={(text) => {
              stopTime.current = text;
            }}
            keyboardType="numeric"
          />
        </View>
        <Button
          style={Style.buttonSave}
          styleText={Style.textButtonSave}
          onPress={handleSave}>
          Salvar
        </Button>
      </View>
    </>
  );
}
