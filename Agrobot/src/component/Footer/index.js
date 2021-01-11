import {View, Text, Image} from 'react-native';
import Styles from './styles';
import React from 'react';

import constLabiotImg from './resources/labiot.png';
import constPtiImg from './resources/pti.png';
import constUnioesteImg from './resources/unioeste.png';
import constItaipuImg from './resources/Itaipu.png';
import constHmccImg from './resources/hmcc.jpeg';
import constReceitaImg from './resources/receitaFederal.png';
import constMunicipalImg from './resources/municipal.png';

export default function footer() {
  return (
    <View View style={Styles.containerLogoVersion}>
      <View style={Styles.logosView}>
        <Image style={Styles.logoReceita} source={constReceitaImg} />
        <Image style={Styles.logoMunicipal} source={constMunicipalImg} />
        <Image style={Styles.logoHmcc} source={constHmccImg} />
      </View>
      <View style={Styles.logosView}>
        <Image style={Styles.logoLabiot} source={constLabiotImg} />
        <Image style={Styles.logoUnioeste} source={constUnioesteImg} />
        <Image style={Styles.logoPti} source={constPtiImg} />
        <Image style={Styles.logoItaipu} source={constItaipuImg} />
      </View>

      <View>
        <Text style={Styles.versionText}>Beta</Text>
      </View>
    </View>
  );
}
