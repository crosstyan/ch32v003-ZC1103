# LLCC68

## Frequency Band

```cpp
/**
 * 载波中心频率 
 *
 * @note 中国-工业和信息化部
 *
 * 《中华人民共和国无线电频率划分规定》(工业和信息化部令第46号令 2018年7月1日起施行) 
 *  规定的无需特别批准ISM应用频段( Sub-1G范围 ):  
 *          6.765—6.795MHz （中心频率6.780MHz）
 *         13.553—13.567MHz (中心频率13.560MHz)
 *         26.957—27.283MHz (中心频率27.120MHz)     
 *          40.66—40.7MHz   (中心频率40.68MHz）   
 *
 * 《微功率(短距离)无线电设备的技术要求》（信部无〔2005〕423号）(工业和信息化部第52号文 2019年)
 *  规定的微功率无线电设备应用频段( Sub-1G范围 ):
 *  A类      9KHz—190KHz
 *  B类    1.7MHz—2.1MHz 
 *         2.2MHz—3.0MHz 
 *         3.1MHz—4.1MHz 
 *         4.2MHz—5.6MHz 
 *         5.7MHz—6.2MHz 
 *         7.3MHz—8.3MHz 
 *         8.4MHz—9.9MHz 
 *  C类  6.765MHz—6.795MHz 
 *      13.553MHz—13.567MHz 
 *      26.957MHz—27.283MHz 
 *  D类    315kHz—30MHz 
 *  E类  40.66MHz—40.7MHz
 *  ------------------------- 
 *      26.975MHz—27.255MHz    模型、玩具: 海摸/车模
 *       40.61MHz—40.75MHz     模型、玩具: 海摸/车模
 *       40.77MHz—40.85MHz     模型、玩具: 空模
 *          45MHz—45.475MHz    模拟式电话机: 座机类
 *          48MHz—48.475MHz    模拟式电话机: 手机类
 *       72.13MHz—72.84MHz     模型、玩具: 空模
 *        75.4MHz—76.0MHz      声音传输及计量仪表
 *          84MHz—87MHz        声音传输及计量仪表
 *          87MHz—108MHz       声音传输及计量仪表
 *         174MHz—216MHz       生物医学设备 
 *       189.9MHz—223.0MHz     声音传输及计量仪表
 *         223MHz—224MHz       电子吊秤专用
 *         224MHz—231MHz       无线数据传送设备
 *         314MHz—316MHz       民用设备控制
 *         407MHz—425MHz       生物医学设备
 *      409.75MHz—409.9875MHz  公众对讲机
 *     419.950MHz—419.275MHz   工业用无线遥控设备
 *         430MHz—432MHz       民用设备控制
 *         433MHz—434.79MHz    民用设备控制
 *    450.0125MHz—450.2125MHz  民用设备控制
 *         470MHz—510MHz       民用计量设备
 *         470MHz—566MHz       通用无线遥控设备
 *         608MHz—630MHz       生物医学设备
 *         614MHz—787MHz       通用无线遥控设备
 *                
 * @note 联合国-国际电信联盟-无线电通信部门(ITU-R)
 * 
 * 《Report ITU-R SM.2180: Impact of Industrial, Scientific and Medical (ISM) Equipment on Radiocommunication Services》(2011年1月)
 *  SM.2180报告指定无需特别批准的ISM应用频段( Sub-1G范围 ): 
 *          6.765—6.795MHz （中心频率6.780MHz）
 *         13.553—13.567MHz (中心频率13.560MHz)
 *         26.957—27.283MHz (中心频率27.120MHz)   
 *          40.66—40.7MHz   (中心频率40.68MHz）      
 *         433.05—434.79MHz (中心频率433.92MHz）   限制在ITU 1区使用 (欧洲和非洲以及蒙古、原苏联以北的地区和欧洲、非两洲以外原苏联及土耳其的领土)
 *            902—928MHz    (中心频率915MHz）      限制在ITU 2区使用 (南、北美洲和夏威夷)       
 *  中国在ITU 3区(亚洲、大洋洲和两洲以外的伊朗领土)
 * 
 * @note 欧盟
 *  《COMMISSION DECISION of 9 November 2006 - on harmonisation of the radio spectrum for use by short-range devices》(2006/771/EC)
 *   欧盟无线电开放频段中的：无需特别批准频段还包含有：
 *          868.0—869.25MHz 
 *
 *   如需CE认证，请了解欧盟无线设备指令RED 2014/53/EU 
 *
 * @note 美国 
 *  《47 CFR Part 18 - INDUSTRIAL, SCIENTIFIC, AND MEDICAL EQUIPMENT》
 *   FCC第18部分规则规定了ISM频段 
 *
 *   如需FCC认证，请了解 47 CFR Part 15
 */
```
