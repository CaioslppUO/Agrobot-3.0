import {View, Text, Dimensions} from 'react-native';
import React from 'react';
import Slider from '@react-native-community/slider';

import Joystick from '../../component/Joystick';
import Footer from '../../component/Footer';
import ConnectedIndicator from '../../component/ConnectedIndicator';
import Button from '../../component/Button';
import ButtonIcon from '../../component/buttonIcon';

import {DataContext} from '../../context/dataContext';
import {CommunicationContext} from '../../context/communication';

import Style from './style.js';

const heightScreen = Dimensions.get('window').height;
export default function Control({navigation}) {
  const {
    setSteer,
    setSpeed,
    setLimit,
    limit,
    handleModuleRobotSwitch,
    handlePowerSwitch,
    handleAutoModeSwitch,
    power,
    moduleRobot,
    autoMode,
    setAutoMode,
  } = React.useContext(DataContext);

  const {connected} = React.useContext(CommunicationContext);
  function handleSetLimitPlus() {
    if (limit < 100) {
      setLimit(limit + 1);
    }
  }
  function handleSetLimitMinus() {
    if (limit > 0) {
      setLimit(limit - 1);
    }
  }
  function handleStopButton() {
    setAutoMode(false);
  }
  return (
    <>
      <View style={Style.container}>
        <View style={Style.header}>
          <ButtonIcon
            name="bars"
            size={25}
            onPress={navigation.openDrawer}
            style={Style.buttonMenu}
          />
          <ConnectedIndicator connected={connected} />
        </View>
        <View style={Style.joystickContainer}>
          <Joystick
            size={heightScreen * 0.21}
            handlerSize={heightScreen * 0.14}
            wrapperStyle={{backgroundColor: '#fff'}}
            handlerStyle={{backgroundColor: '#000'}}
            resetOnRelease={true}
            autoCenter={false}
            onValue={({x, y}) => {
              setSpeed(Number(x * 100).toPrecision(3));
              setSteer(Number(y * 100).toPrecision(3));
            }}
          />
        </View>
        <View style={Style.internalContainer}>
          <View style={Style.buttonsContainer}>
            <ButtonIcon
              name="power-off"
              size={30}
              color={power ? '#0f0' : '#ff6666'}
              onPress={handlePowerSwitch}
              style={Style.buttonIcon}
            />
            <ButtonIcon
              name="puzzle-piece"
              size={30}
              color={moduleRobot ? '#0f0' : '#ff6666'}
              onPress={handleModuleRobotSwitch}
              style={Style.buttonIcon}
            />
            <ButtonIcon
              name="car"
              size={30}
              color={autoMode ? '#0f0' : '#000'}
              onPress={handleAutoModeSwitch}
              style={Style.buttonIcon}
            />
          </View>
          <Text>Velocidade: {limit}</Text>
          <View style={Style.sliderContainer}>
            <ButtonIcon name="minus" size={15} onPress={handleSetLimitMinus} />
            <Slider
              maximumValue={100}
              minimumValue={0}
              value={limit}
              onValueChange={(sliderValue) => {
                setLimit(sliderValue);
              }}
              style={Style.slider}
              step={1}
            />
            <ButtonIcon name="plus" size={15} onPress={handleSetLimitPlus} />
          </View>
          <Button
            style={Style.buttonStop}
            styleText={Style.textButtonStop}
            onPress={handleStopButton}>
            Parar
          </Button>
        </View>
        <Footer />
      </View>
    </>
  );
}
