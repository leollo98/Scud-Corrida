#include <Arduino.h>
#include <doxygen.h>
#include <NexButton.h>
#include <NexCheckbox.h>
#include <NexConfig.h>
#include <NexCrop.h>
#include <NexDualStateButton.h>
#include <NexGauge.h>
#include <NexGpio.h>
#include <NexHardware.h>
#include <NexHotspot.h>
#include <NexNumber.h>
#include <NexObject.h>
#include <NexPage.h>
#include <NexPicture.h>
#include <NexProgressBar.h>
#include <NexRadio.h>
#include <NexRtc.h>
#include <NexScrolltext.h>
#include <NexSlider.h>
#include <NexText.h>
#include <NexTimer.h>
#include <Nextion.h>
#include <NexTouch.h>
#include <NexUpload.h>
#include <NexVariable.h>
#include <NexWaveform.h>
#include <SPI.h>

#define debug

#define LedPit 		13
#define LedRPM 		12
#define LedPcomb 	11
#define LedPoleo 	10
#define LedToleo	9
#define Alavanca	8
#define BotaoR		7
#define BotaoL		6
#define LuzFreio 	A0
#define RBPi		Serial


		byte	inbyte = 0;
		byte	enrich = 0;
		byte 	inData[140]; // Allocate some space for the string
const	int 	chipSelect = 53;
const	int 	valor = -1;

		int		index = 0; // Index into array; where to store the character
		int 	ase = 0;
		int 	running = 0;
		int 	wue1 = 0;
		int 	idle = 0;
		int 	accel = 0;
		int 	decel = 0;
		int 	waitsd = 0;
		int 	sdstop = 0;

signed 	int 	accelenrich = 0;
signed	int 	mapdot = 0;
signed 	int 	afrnew = 0;

unsigned int 	gair = 0;
unsigned int 	mat = 0;
unsigned int 	wue = 0;
unsigned int 	gamma = 0;	
unsigned int 	clt = 0;
unsigned int 	ve = 0;
unsigned int 	ve2 = 0;
unsigned int 	kpa1 = 0;
unsigned int 	pulsewidth1 = 0;
unsigned int 	pulsewidth2 = 0;
unsigned int 	rpm1 = 0;
unsigned int 	xtime1 = 0;
unsigned int 	xtime2 = 0;
unsigned int 	afrt1 = 0;
unsigned int 	egocorrect = 0;
unsigned int 	adv1 = 0;
unsigned int 	batt1 = 0;
unsigned int 	synccnt = 0; //94
unsigned int 	syncreason = 0; //139
unsigned int 	squirt = 0;
unsigned int 	bar1 = 0;

		float 	mat1 = 0;
		float 	accelenrich1 = 0;
		float 	clt1 = 0;
		float 	kpa2 = 0;
		float 	ve1 = 0;
		float 	ve3 = 0;
		float 	pulsewidth2c = 0;
		float	pulsewidth1c = 0;
		float 	afrt2 = 0;
		float 	adv2 = 0;
		float 	afrnewf = 0;
		float 	batt2 = 0;
		float 	bar2 = 0;
		float	tps = 0;
		float	egoV1 = 0;
		float	fuelload =0;


//Texto no Nextion
NexText t0 = NexText(0, 11, "t0");
NexText t1 = NexText(0, 2, "t1");
NexText t2 = NexText(0, 10, "t2");
NexText t3 = NexText(0, 3, "t3");
NexText t4 = NexText(0, 4, "t4");
NexText t8 = NexText(0, 8, "t8");
NexText t9 = NexText(0, 12, "t9");
// Páginas do Nextion
NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");

char  txt0[10],
      txt1[10],
      txt2[10],
      txt3[10],
      txt4[10],
      txt8[10],
      txt9[10];

int marcha=0,
    pot_value = 0,
    page=0,
    alert=0;

boolean botaoAnt = false,
        botaoAtu = false;


void processserialdata(){
    xtime1 = (inData[0] << 8) | (inData[1]);
    //xtime2 = xtime1/256;
    pulsewidth1 = (inData[2] << 8) | (inData[3]);
    pulsewidth1c = pulsewidth1 * 0.000666;
    //  pulsewidth1=pulsewidth1/1500;
	pulsewidth2 = (inData[4] << 8) | (inData[5]);
    pulsewidth2c = pulsewidth2 * 0.000666;
    rpm1 = (inData[6] << 8) | (inData[7]);
    adv1 = (inData[8] << 8) | (inData[9]);
    adv2 = adv1 * 0.1;
    //  adv2 = adv1/10;
	squirt = (inData[10]);
    enrich = (inData[11]);
    afrt1 = (inData[12]);
    afrt2 = afrt1 * 0.1;
    //  afrt2= afrt1/10;
	bar1 = (inData[16] << 8) | (inData[17]);
	bar2 = bar1 * 0.1;
    kpa1 = (inData[18] << 8) | (inData[19]);
    kpa2 = kpa1 * 0.1;
    //  kpa2 = kpa1/10;
    mat = (inData[20] << 8) | (inData[21]);
    mat1 = ((mat - 320) / 18);
    //  mat1 = (0.0555*(mat-320));
    clt = (inData[22] << 8) | (inData[23]);
    clt1 = ((clt - 320) / 18);
    //  clt1 = (0.0555*(clt-320));
	tps = (inData[24] << 8) | (inData[25]);
    batt1 = (inData[26] << 8) | (inData[27]);
    batt2 = batt1 * 0.1;
    //  batt2 = batt1/10;
    afrnew = (inData[28] << 8) | (inData[29]);
    afrnewf = afrnew * 0.1;
    //  afrnewf = afrnew/10;
    egocorrect = (inData[34] << 8) | (inData[35]);
    egocorrect = egocorrect / 10;
    //  egocorrect = egocorrect*0.1;
    gair = (inData[38] << 8) | (inData[39]);
    wue = (inData[40] << 8) | (inData[41]);
    accelenrich = (inData[42] << 8) | (inData[43]);
    accelenrich1 = (accelenrich * 0.1);
    //  accelenrich1=(accelenrich*0.1);
    gamma = (inData[48] << 8) | (inData[49]);
    ve = ((inData[50] << 8) | (inData[51]));
    ve1 = (ve * 0.1);
    //  ve1=(ve/10);
	ve2 = ((inData[52] << 8) | (inData[53]));
    ve3 = (ve * 0.1);
    //  ve3=(ve2/10);
    mapdot = ((inData[60] << 8) | (inData[61]));
	fuelload = (inData[66]);
	egoV1 = ((inData[74] << 8) | (inData[75]));
    accel = bitRead(enrich, 6);
    decel = bitRead(enrich, 7);
    ase = bitRead(enrich, 2);
    wue1 = bitRead(enrich, 3);
    //  idle=bitRead(enrich,7);
    running = bitRead(enrich, 0);
    synccnt = (inData[94]);
    syncreason = (inData[139]);
}

void serialprint(){
    Serial.print("Time:");
    Serial.print(xtime1);
    Serial.print("s ");
    Serial.print("MAP:");
    Serial.print((kpa2));
    Serial.print(" kpa ");
    Serial.print("VE:");
    Serial.print((ve1));
    Serial.print("% ");
    Serial.print("Pulsewidth:");
    Serial.print(pulsewidth2);
    Serial.print("ms ");
    Serial.print("RPM:");
    Serial.print(rpm1);
    Serial.print(" rpm ");
    Serial.print("adv:");
    Serial.print(adv2);
    Serial.print(" deg ");
    Serial.print("Accel:");
    if (accel == 1)
    {
        Serial.print("A ");
    } else if (decel == 1)
    {
        Serial.print("D ");
    } else
    {
        Serial.print("  ");
    }
    Serial.print("IAT:");
    Serial.print(mat1);
    Serial.print(" degC ");
    Serial.print("CLT:");
    Serial.print(clt1);
    Serial.print(" degC ");
    Serial.print("WUE:");
    Serial.print(wue);
    Serial.print("% ");
    Serial.print("Voltage:");
    Serial.print(batt2);
    Serial.print(" V ");
    Serial.print("GAIR:");
    Serial.print(gair);
    Serial.print("% ");
    Serial.print("GAMMA:");
    Serial.print(gamma);
    Serial.print("EGOCorrect:");
    Serial.print(egocorrect);
    Serial.print("% ");
    Serial.print(" AFR Tgt:");
    Serial.print(afrt2);
    Serial.print(" ");
    Serial.print("AFR Actual:");
    Serial.print(afrnewf);
    Serial.println();
}

void sendRBPi(){
	Serial.println(xtime1);	//Seconds
	Serial.println(rpm1);	//RPM
	Serial.println(bar2);	//Barometric pressur
	Serial.println(kpa2);	//Manifold air pressure
	Serial.println(clt1);	//Coolant temperature
	Serial.println(tps);	//Throttle position
	Serial.println(batt2);	//Battery voltage
	Serial.println(gair);	//Air density correction
	Serial.println(wue);	//Warmup correction
	Serial.println(ve1);	//VE value table/bank 1
	Serial.println(ve3);	//VE value table/bank 2
	Serial.println(fuelload);	//MAP in Speed-Density
	//Serial.println();	//AFR cyl#1
	//Serial.println();	//Current gear selected
	Serial.println(egoV1);	//Voltage from O2
	//Serial.println();	//Vehicle Speed 1
	//Serial.println();	//Injection Timing Angle (primary)
	//Serial.println();	//Average fuel consumption
	//Serial.println();	//Fuel pressure 1
	//Serial.println();	//Battery current (alternator system)
}


void setup(){
    Serial.begin(9600); 	//Comunicação com RBPi
    Serial1.begin(115200);	//Comunicação com ES3X via RS232
	Serial2.begin(9600);	//Comunicação com Nextion
	Serial3.begin(9600);	//Comunicação com HC-12
	delay(10);
	Serial.println("arduino OK");

	nexInit();
	page0.show();
	pinMode(BotaoL, INPUT_PULLUP);
	pinMode(BotaoR, INPUT_PULLUP);
	pinMode(Alavanca, INPUT_PULLUP);
	pinMode(LedPit, OUTPUT);
	pinMode(LedPoleo, OUTPUT);
	pinMode(LedPcomb, OUTPUT);
	pinMode(LedRPM, OUTPUT);
	pinMode(LedToleo, OUTPUT);
}

//Alerta
void Alert(int page, int val, int maximo, NexText t){
    if(val>=maximo){
        page2.show();
        t.Set_font_color_pco(2047);
        alert=1;
    }
    else{
        if(alert == 1){
            if(page % 2 == 0)
                page1.show();
            else
                page0.show();
            alert=0;
        }   
    }
}

void loop(){
	//-----------------------------RS232---------------------------------
    index = 0;
    Serial1.write("A"); // calling data from megasquirt. Array of 169 items to be received
    while (Serial1.available() < 20)
    {
        //waiting for buffer to fill;
    }
    Serial.println(Serial1.available());
    if (Serial1.available() > 0)
    {
        delay(15);
        for (index = 0; index < 140; index++) //trying 70 elements
        {
            inData[index] = Serial1.read(); // Read a character
            //  delay(1); //is delay necessary?
        }
        Serial1.flush();
        processserialdata();
		#ifdef debug
			serialprint();
		#endif
        
        inData[index] = '\0'; // Null terminate the strinG
    }
	//-----------------------------Leds----------------------------------

    if(rpm1>=5000)
        digitalWrite(LedRPM,1);
    else
        digitalWrite(LedRPM,0);

    if(rpm1>=5000)
        digitalWrite(LedPoleo,1);
    else
        digitalWrite(LedPoleo,0);

    if(rpm1>=5000)
        digitalWrite(LedPcomb,1);
    else
        digitalWrite(LedPcomb,0);

    if(rpm1>=5000)
        digitalWrite(LedToleo,1);
    else
        digitalWrite(LedToleo,0);

    if(!digitalRead(Alavanca))
        digitalWrite(LedPit,1);
    else
        digitalWrite(LedPit,0);

	//-----------------------------Nextion-------------------------------
	// Botão para mudar de página
  	botaoAtu = !digitalRead(BotaoL);
  	if(botaoAtu && !botaoAnt){
    	if(page % 2 == 0)
      		page1.show();
    	else
      		page0.show();
    	page++;
  	}
	  botaoAnt=botaoAtu;

      //Mudanças no Texto
    t0.setText(txt0);
    t1.setText(txt1);
    t2.setText(txt2);
    t3.setText(txt3);
    t4.setText(txt4);
    t8.setText(txt8);
    t9.setText(txt9);

    memset(txt0, 0, sizeof(txt0));
    itoa(kpa2, txt0, 10);
    Alert(page,kpa2,500,t0);

    memset(txt1, 0, sizeof(txt1));
    itoa(mat1, txt1, 10);
    Alert(page,mat1,500,t1);

    memset(txt2, 0, sizeof(txt2));
    itoa(clt1, txt2, 10);
    Alert(page,clt1,500,t2);

    memset(txt3, 0, sizeof(txt3));
    itoa(rpm1, txt3, 10);
    Alert(page,rpm1,500,t3);

    memset(txt4, 0, sizeof(txt4));
    itoa(1, txt4, 10);
    Alert(page,1,500,t4);

    memset(txt8, 0, sizeof(txt8));
    itoa(1, txt8, 10);
    Alert(page,1,7,t8);

    memset(txt9, 0, sizeof(txt9));
    itoa(batt2, txt9, 10);
    Alert(page,batt2,16,t9);
	
    delay(10);
}