# co2-detect
Open source project for a simple CO2 sensor using MQ135

# The MQ135 sensor
MQ135 is a general purpose gas detection sensor intented to be used to assess air quality. It detects a wide range of gases being, thus, very unespecific.

One of the not-so-bad datasheets that can be found on the web is [this one](https://www.electronicoscaldas.com/datasheet/MQ-135_Hanwei.pdf). By not-so-bad I meant it is at least readable, but nevertheless lacks fundamental information to setup the sensor correctly. To be more specific, it is not clear about how the sensor sensitivity graphic was generated.

But, before getting there, let's look at the eletronics involved.

## MQ135 functioning aspects
MQ135 sense gas concentrations by using a tin dioxide (Sn0<sub>2</sub>) layer deposited on a alumina base (Al<sub>2</sub>O<sub>3</sub>). Just for the record, tin oxide is begin considered as a CO<sub>2</sub> harvesting substance that can eventually help removing this gas from the atmosphere, [check it out](https://doi.org/10.1021/acsaem.8b02048).

![MQ135 sensitivity curve](https://user-images.githubusercontent.com/18699508/116015668-76568000-a610-11eb-85e7-664996fa7023.png)
*Figure: MQ135 sensitivity curve from its datasheet*

There is an extensive bibliography on Sn0<sub>2</sub> sensors, and some of it can be consulted by the end of this README. The basic mechanism consists of an interaction of the Sn0<sub>2</sub> - which is a semiconductor - cristals with the incoming gases (GÖPEL, SCHIERBAUM, 1995). A set of surface and grain boundary reactions occur, changing overall conductivity. This can be measured as a change on resistance, and a relationship between resistance and gas concentration can be achieved. Sensors using Sn0<sub>2</sub> are very cheap e reliable, but with low specificity, and highly dependent on temperature, humidity, and on the composition of the surrounding air. This last depence is the reason why manufacturers recommend that O<sub>2</sub> concentration must be at 21%.

The Sn0<sub>2</sub> layer must be heated up to achieve optimal response characteristics, and most references put this temperature at around 400 °C, which is pretty high for a small, low cost sensor. Recent researches are aiming at developing room temperature sensors, but it seems unlinkely our little, cheap MQ135 is a state-of-the-art device. Thus, the sensor needs to be heated, and that's why there is a small 33 ohms resistance that must be energized. Heating times should be high enough for the sensor resistance to stabilize, and most datasheet recommend from 24 h to 48 h of pre heating before measuring can be done.

## MQ135 eletric circuit

MQ135 should be assemble in a voltage divider circuit

![image](https://user-images.githubusercontent.com/18699508/116021508-b3297380-a61e-11eb-85e1-623e7e8ac4dc.png)

It is easy to see that the relationship between the output voltage and the sensor resistance is given by ![image](https://user-images.githubusercontent.com/18699508/116022122-f506e980-a61f-11eb-8073-6a54fe5ef5c0.png)

## The sensor equation

Before moving into the equation for the gas concentration, we should first notice that the curves provided by the datasheet are all based on a R<sub>0</sub> resistance, which is the resistance measured at 100 ppm of NH<sub>3</sub>, no other contaminants present. The ratio between the resistance measured with clean air and R<sub>0</sub> is approximatelly 3.6. 

Here we come at the first information problem at the datasheet. It is not clear whether the measured quantities (abscissa) are the **absolute values** or the **relative values** in clean air. For most toxic gases this question is irrelevant, because their concentrations in the air we breath is very small (methane appears at [roughly 1.7 ppm](https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html), while carbon monoxide and ammonia concentrations are around 100 parts per **billion**). CO<sub>2</sub>, however, is unfortunately more and more present: 410 ppm.As we can see, in the graphic the maximum concentration is 200 ppm and, then, it seems logical to suppose that the concentrations are presented as relative to standard clean air.

Antoher thing that is apparent from the curves, and which is confirmed by the measeurements performed by the works on our References sections, is that the relationship between resistance and concentration, at least up to 200 ppm, is an exponentional one. That means that the concentration *C* can be found as:

![image](https://user-images.githubusercontent.com/18699508/116023901-80ce4500-a623-11eb-86e5-75f1adeb1ff5.png)

By using the points available in the graph, one can find that b = 116 ppm and m = -2.993 for carbon dioxide. Whether these numbers are true and provide exact concentration readings is a whole other story, of course. But let's assume they are right!

Let's call the sensor resistance in clean air R<sub>c</sub>. Well, we've already verified that R<sub>c</sub>/R<sub>0</sub>=3.6, or, conversely, R<sub>0</sub>=R<sub>c</sub>/3.6. We can then modify our previous exponential relationship between concentration and measured resistance as 

![image](https://user-images.githubusercontent.com/18699508/116024397-7eb8b600-a624-11eb-92dd-2c3146b29be6.png)

Finally, because we are assuming that our measurement is relative to clean atmospheric air, the absolute concentration of CO<sub>2</sub> measured is

![image](https://user-images.githubusercontent.com/18699508/116024508-ba538000-a624-11eb-83cb-e685efd368a5.png)

# Important limitations
Other types of detectable gases **must be absent** from the measured environment. Their presence will affect the values obtained.

Air umidity amd temperature corrections are no yet implemented here.

# CO<sub>2</sub> Data
Global CO2 concentrations are measured worldwide and condensed on the [Global Monitoring Laboratory](https://www.esrl.noaa.gov/gmd/). In particular, global trends on several latitudes are shown on the followng graphic:
![alt text](https://www.esrl.noaa.gov/gmd/webdata/ccgg/trends/global_trend.png "Daily Global CO2")
From the graphic, it can be seen that the average CO2 concentration on the atmosphere is around 410 ppm with small variations along the year on tempered regions.

# References

[1]D. Wang et al., “CO2-sensing properties and mechanism of nano-SnO2 thick-film sensor,” Sensors and Actuators B: Chemical, vol. 227, pp. 73–84, May 2016, doi: 10.1016/j.snb.2015.12.025.

[2]R. C. Abruzzi et al., “Application of SnO2 Nanoparticles and Zeolites in Coal Mine Methane Sensors,” Materials Research, vol. 22, 2019, doi: 10.1590/1980-5373-mr-2018-0818. [Online]. Available: http://www.scielo.br/scielo.php?script=sci_abstract&pid=S1516-14392019000700204&lng=en&nrm=iso&tlng=en. [Accessed: 26-Apr-2021]

[3]W. Göpel and K. D. Schierbaum, “SnO2 sensors: current status and future prospects,” Sensors and Actuators B: Chemical, vol. 26, no. 1–3, pp. 1–12, Jan. 1995, doi: 10.1016/0925-4005(94)01546-T. 

[4]S. Hahn, “SnO2 thick film sensors at ultimate limits,” Master’s Dissertation, Eberhard-Karls-Universität Tübingen, Tübingen, 2002 [Online]. Available: https://publikationen.uni-tuebingen.de/xmlui/bitstream/handle/10900/48386/pdf/Diss_SimoneHahn.pdf?sequence=1&isAllowed=y



