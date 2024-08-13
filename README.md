# Ротатор для Panoptix на Arduino
Программа для управления шаговым двигателем <b>NEMA 23</b> с помощью драйвера <b>TB6600 </b> 
или совместимого. Драйвер подключается по четырехпроводной схеме  
&emsp;&emsp;__ENA__ - Enable Разрешает работу мотора  
&emsp;&emsp;__DIR__ - Direction Направление вращения  
&emsp;&emsp;__PUL__ - Pulse Тактовые импульсы  
Для корректной работы программы на драйвере должен быть установлен режим 3200 импульсов на оборот.<br> Для TB6600 (S1-OFF, S2-OFF, S3-ON).  
Для управления шаговым двигателем используется библиотека [GyverStepper](https://github.com/GyverLibs/GyverStepper)

  Управление осуществляется при помощи 4 кнопок и 2 потенциометров 10кОм.
- __CW__ вращение с ускорением по чвсовой стрелке. При нажатии, вал двигателя
начинает вращение по часовой стрелке с ускорением 150 шаг/сек до достижении
скорости, установленной потенциометром __SPEED__ Максимальный угол поворота
при удержании кнопки 360&deg; При нажатии происходит выход из режима сканирования.
- __CCW__ вращение с ускорением против чвсовой стрелки. При нажатии, вал двигателя
начинает вращение против часовой стрелки с ускорением 150 шаг/сек до достижении
скорости, установленной потенциометром __SPEED__ Максимальный угол поворота
при удержании кнопки 360&deg; При нажатии происходит выход из режима сканирования.
- __CW + CCW__ одновременное нажатие активирует режим сканирования.
- __SCAN__ запускает режим сканирования. Сектор сканирования задается
потенциометром __ANGLE__. При нажатии в режиме сканирования, задается
новая центральная точка сканирования. 
- __BREAKE__ включает/отключает режим удержания вала двигателя.
- __SPEED__ изменяет скорость вращения и сканирования.
- __ANGLE__ изменяет сектор сканирования 10&deg; - 360&deg;<br>
<br>
Для приема сигналов от радиопультов используется приемник <b>SYN480R</b> и библиотека <a href="https://github.com/sui77/rc-switch/tree/master">RC-Switch</a> <br>
<h2>Программирование пультов</h2>
В память могут быть занесены три двухкнопочных пульта. Алгоритм работы пультов аналогичен работе кнопок.<br>
При одновременном нажатии двух кнопок включается режим сканирования.<br>
Для <b>записи</b> кодов пульта в память контроллера необходимо выполнить следующие действия:<br>
1. Отключить питание <br>
2. Зажать кнопку BREAK и подать питание<br>
3. Дождаться пока светодиод на плате начнет мигать. Одна выспышка сигнализирует о готовности к записи<br>
&nbsp &nbsp кода первой кнопки. Нажать кнопку на пульте, отвечающий за поворотпротив часовой стрелки<br>
4. Когда светодиод начнет мигать по два раза, нажать вторую кнопку пульта<br>
5. Три вспышки светодиода сообщают о готовности записи кода команды "сканирование". Нажать обе кнопки<br>
&nbsp &nbsp на пульте одновременно.<br>
6. Пульт готов к работе.<br>
Для записи дополнительных пультов повторить пункты 1-6. Нельзя записать одну кнопку дважды. Автоматический<br>
выход из режима программирования происходит через 30 секунд. При записи четвертого пульта, первый ситрается<br>
из памяти (FIFO).<br>
Для <b>удаления</b> кодов пультов необходимо:<br>
1. Отключить питание <br>
2. Зажать кнопку BREAK и подать питание<br>
3. Дождаться пока светодиод на плате начнет мигать.<br>
4. Продолжать удерживать нажатой кнопку BREAK в течение 5 секунд, пока светодиод не загорится постоянным <br>
&nbsp &nbsp светом.<br>
5. Отпустить кнопку BREAK <br>
6. Коды всех пультов удалены из памяти <br>

<h2>Видео</h2>
Ссылка на <a href="https://youtu.be/_9E2vSRK5No">YouTube</a> <br>
Ссылка на <a href="https://dzen.ru/video/watch/66b5f1579cfcc32754c732f5">Дзен</a> <br>

<h2>Применимость</h2>
Может быть использован для системы управления вращением датчиков таких как<br>
<B>Garmin Panoptix</B> или <B>Lowrance Active Target</B>  
 
<h2>Совместимость</h2>
Совместима со всеми Arduino платформами (используются Arduino-функции)  
<h2>Схема соединений</h2>  
  
![alt text](https://github.com/BalandinSV/ABPM8-firmware/blob/main/Wiring%20diagram%20RF.png)
