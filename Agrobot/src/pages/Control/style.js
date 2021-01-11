import {StyleSheet, Dimensions} from 'react-native';

const widthScreen = Dimensions.get('window').width;
const heightScreen = Dimensions.get('window').height;

export default StyleSheet.create({
  container: {
    flex: 1,
    alignItems: 'center',
    flexDirection: 'column',
    justifyContent: 'space-between',
    backgroundColor: '#fff',
  },
  buttonStop: {
    borderColor: '#f00',
    borderRadius: 115,
    height: 62,
    width: 200,
    borderWidth: 3,
    backgroundColor: '#ff4444',
  },
  textButtonStop: {
    fontSize: 15,
    textAlign: 'center',
    color: '#fff',
  },
  slider: {
    width: widthScreen * 0.75,
  },
  sliderContainer: {
    height: heightScreen * 0.05,
    width: widthScreen,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-evenly',
    backgroundColor: '#fff',
  },
  internalContainer: {
    height: heightScreen * 0.3,
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '1%',
    backgroundColor: '#fff',
  },
  buttonsContainer: {
    flexDirection: 'row',
    justifyContent: 'space-evenly',
    padding: '2%',
    width: widthScreen,
    backgroundColor: '#fff',
  },
  buttonIcon: {
    width: 60,
    height: 60,
  },
  buttonMenu: {
    borderWidth: 0,
    marginLeft: 8,
    marginTop: 8,
  },
  header: {
    width: widthScreen,
    alignItems: 'center',
    justifyContent: 'space-between',
    flexDirection: 'row',
    backgroundColor: '#fff',
  },
  joystickContainer: {
    backgroundColor: '#fff',
    height: heightScreen * 0.42,
    alignItems: 'center',
    justifyContent: 'center',
  },
});
