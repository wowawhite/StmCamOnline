/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
//#include "stm32f4xx_hal.h"
//#include "arm_neon.h"  // no neon for cortex-m4
#include "stddef.h"
//#include <stm32f4xx_hal_dma_ex.h>  // dma extended functions
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
// those are for ETHERNET
#include "loopback.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500.h"
// those are for LCD + Touch
#include "ILI9341_STM32_Driver.h"
//#include "ILI9341_GFX.h"
//#include "snow_tiger.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MODE

#define DATA_SOCK	0
//#define CONTROL_SOCK	1  // not used
#define CAM_BUF_SIZE 13176
#define CAM_FLOAT_SIZE 3200  // 3200 floats, header 94 floats
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#ifdef DEBUG_MODE:
	#define NETWORK_MSG  		 "Network configuration:\r\n"
	#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
	#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
	#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
	#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"

	#define PRINT_NETINFO(netInfo) do { 																					\
	  HAL_UART_Transmit(&huart2, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
	  sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	  sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	  sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	  sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);															\
	} while(0)
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 13176 chars
//char *teststring = "a0123456789101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354555657585960616263646566676869707172737475767778798081828384858687888990919293949596979899100101102103104105106107108109110111112113114115116117118119120121122123124125126127128129130131132133134135136137138139140141142143144145146147148149150151152153154155156157158159160161162163164165166167168169170171172173174175176177178179180181182183184185186187188189190191192193194195196197198199200201202203204205206207208209210211212213214215216217218219220221222223224225226227228229230231232233234235236237238239240241242243244245246247248249250251252253254255256257258259260261262263264265266267268269270271272273274275276277278279280281282283284285286287288289290291292293294295296297298299300301302303304305306307308309310311312313314315316317318319320321322323324325326327328329330331332333334335336337338339340341342343344345346347348349350351352353354355356357358359360361362363364365366367368369370371372373374375376377378379380381382383384385386387388389390391392393394395396397398399400401402403404405406407408409410411412413414415416417418419420421422423424425426427428429430431432433434435436437438439440441442443444445446447448449450451452453454455456457458459460461462463464465466467468469470471472473474475476477478479480481482483484485486487488489490491492493494495496497498499500501502503504505506507508509510511512513514515516517518519520521522523524525526527528529530531532533534535536537538539540541542543544545546547548549550551552553554555556557558559560561562563564565566567568569570571572573574575576577578579580581582583584585586587588589590591592593594595596597598599600601602603604605606607608609610611612613614615616617618619620621622623624625626627628629630631632633634635636637638639640641642643644645646647648649650651652653654655656657658659660661662663664665666667668669670671672673674675676677678679680681682683684685686687688689690691692693694695696697698699700701702703704705706707708709710711712713714715716717718719720721722723724725726727728729730731732733734735736737738739740741742743744745746747748749750751752753754755756757758759760761762763764765766767768769770771772773774775776777778779780781782783784785786787788789790791792793794795796797798799800801802803804805806807808809810811812813814815816817818819820821822823824825826827828829830831832833834835836837838839840841842843844845846847848849850851852853854855856857858859860861862863864865866867868869870871872873874875876877878879880881882883884885886887888889890891892893894895896897898899900901902903904905906907908909910911912913914915916917918919920921922923924925926927928929930931932933934935936937938939940941942943944945946947948949950951952953954955956957958959960961962963964965966967968969970971972973974975976977978979980981982983984985986987988989990991992993994995996997998999100010011002100310041005100610071008100910101011101210131014101510161017101810191020102110221023102410251026102710281029103010311032103310341035103610371038103910401041104210431044104510461047104810491050105110521053105410551056105710581059106010611062106310641065106610671068106910701071107210731074107510761077107810791080108110821083108410851086108710881089109010911092109310941095109610971098109911001101110211031104110511061107110811091110111111121113111411151116111711181119112011211122112311241125112611271128112911301131113211331134113511361137113811391140114111421143114411451146114711481149115011511152115311541155115611571158115911601161116211631164116511661167116811691170117111721173117411751176117711781179118011811182118311841185118611871188118911901191119211931194119511961197119811991200120112021203120412051206120712081209121012111212121312141215121612171218121912201221122212231224122512261227122812291230123112321233123412351236123712381239124012411242124312441245124612471248124912501251125212531254125512561257125812591260126112621263126412651266126712681269127012711272127312741275127612771278127912801281128212831284128512861287128812891290129112921293129412951296129712981299130013011302130313041305130613071308130913101311131213131314131513161317131813191320132113221323132413251326132713281329133013311332133313341335133613371338133913401341134213431344134513461347134813491350135113521353135413551356135713581359136013611362136313641365136613671368136913701371137213731374137513761377137813791380138113821383138413851386138713881389139013911392139313941395139613971398139914001401140214031404140514061407140814091410141114121413141414151416141714181419142014211422142314241425142614271428142914301431143214331434143514361437143814391440144114421443144414451446144714481449145014511452145314541455145614571458145914601461146214631464146514661467146814691470147114721473147414751476147714781479148014811482148314841485148614871488148914901491149214931494149514961497149814991500150115021503150415051506150715081509151015111512151315141515151615171518151915201521152215231524152515261527152815291530153115321533153415351536153715381539154015411542154315441545154615471548154915501551155215531554155515561557155815591560156115621563156415651566156715681569157015711572157315741575157615771578157915801581158215831584158515861587158815891590159115921593159415951596159715981599160016011602160316041605160616071608160916101611161216131614161516161617161816191620162116221623162416251626162716281629163016311632163316341635163616371638163916401641164216431644164516461647164816491650165116521653165416551656165716581659166016611662166316641665166616671668166916701671167216731674167516761677167816791680168116821683168416851686168716881689169016911692169316941695169616971698169917001701170217031704170517061707170817091710171117121713171417151716171717181719172017211722172317241725172617271728172917301731173217331734173517361737173817391740174117421743174417451746174717481749175017511752175317541755175617571758175917601761176217631764176517661767176817691770177117721773177417751776177717781779178017811782178317841785178617871788178917901791179217931794179517961797179817991800180118021803180418051806180718081809181018111812181318141815181618171818181918201821182218231824182518261827182818291830183118321833183418351836183718381839184018411842184318441845184618471848184918501851185218531854185518561857185818591860186118621863186418651866186718681869187018711872187318741875187618771878187918801881188218831884188518861887188818891890189118921893189418951896189718981899190019011902190319041905190619071908190919101911191219131914191519161917191819191920192119221923192419251926192719281929193019311932193319341935193619371938193919401941194219431944194519461947194819491950195119521953195419551956195719581959196019611962196319641965196619671968196919701971197219731974197519761977197819791980198119821983198419851986198719881989199019911992199319941995199619971998199920002001200220032004200520062007200820092010201120122013201420152016201720182019202020212022202320242025202620272028202920302031203220332034203520362037203820392040204120422043204420452046204720482049205020512052205320542055205620572058205920602061206220632064206520662067206820692070207120722073207420752076207720782079208020812082208320842085208620872088208920902091209220932094209520962097209820992100210121022103210421052106210721082109211021112112211321142115211621172118211921202121212221232124212521262127212821292130213121322133213421352136213721382139214021412142214321442145214621472148214921502151215221532154215521562157215821592160216121622163216421652166216721682169217021712172217321742175217621772178217921802181218221832184218521862187218821892190219121922193219421952196219721982199220022012202220322042205220622072208220922102211221222132214221522162217221822192220222122222223222422252226222722282229223022312232223322342235223622372238223922402241224222432244224522462247224822492250225122522253225422552256225722582259226022612262226322642265226622672268226922702271227222732274227522762277227822792280228122822283228422852286228722882289229022912292229322942295229622972298229923002301230223032304230523062307230823092310231123122313231423152316231723182319232023212322232323242325232623272328232923302331233223332334233523362337233823392340234123422343234423452346234723482349235023512352235323542355235623572358235923602361236223632364236523662367236823692370237123722373237423752376237723782379238023812382238323842385238623872388238923902391239223932394239523962397239823992400240124022403240424052406240724082409241024112412241324142415241624172418241924202421242224232424242524262427242824292430243124322433243424352436243724382439244024412442244324442445244624472448244924502451245224532454245524562457245824592460246124622463246424652466246724682469247024712472247324742475247624772478247924802481248224832484248524862487248824892490249124922493249424952496249724982499250025012502250325042505250625072508250925102511251225132514251525162517251825192520252125222523252425252526252725282529253025312532253325342535253625372538253925402541254225432544254525462547254825492550255125522553255425552556255725582559256025612562256325642565256625672568256925702571257225732574257525762577257825792580258125822583258425852586258725882589259025912592259325942595259625972598259926002601260226032604260526062607260826092610261126122613261426152616261726182619262026212622262326242625262626272628262926302631263226332634263526362637263826392640264126422643264426452646264726482649265026512652265326542655265626572658265926602661266226632664266526662667266826692670267126722673267426752676267726782679268026812682268326842685268626872688268926902691269226932694269526962697269826992700270127022703270427052706270727082709271027112712271327142715271627172718271927202721272227232724272527262727272827292730273127322733273427352736273727382739274027412742274327442745274627472748274927502751275227532754275527562757275827592760276127622763276427652766276727682769277027712772277327742775277627772778277927802781278227832784278527862787278827892790279127922793279427952796279727982799280028012802280328042805280628072808280928102811281228132814281528162817281828192820282128222823282428252826282728282829283028312832283328342835283628372838283928402841284228432844284528462847284828492850285128522853285428552856285728582859286028612862286328642865286628672868286928702871287228732874287528762877287828792880288128822883288428852886288728882889289028912892289328942895289628972898289929002901290229032904290529062907290829092910291129122913291429152916291729182919292029212922292329242925292629272928292929302931293229332934293529362937293829392940294129422943294429452946294729482949295029512952295329542955295629572958295929602961296229632964296529662967296829692970297129722973297429752976297729782979298029812982298329842985298629872988298929902991299229932994299529962997299829993000300130023003300430053006300730083009301030113012301330143015301630173018301930203021302230233024302530263027302830293030303130323033303430353036303730383039304030413042304330443045304630473048304930503051305230533054305530563057305830593060306130623063306430653066306730683069307030713072307330743075307630773078307930803081308230833084308530863087308830893090309130923093309430953096309730983099310031013102310331043105310631073108310931103111311231133114311531163117311831193120312131223123312431253126312731283129313031313132313331343135313631373138313931403141314231433144314531463147314831493150315131523153315431553156315731583159316031613162316331643165316631673168316931703171317231733174317531763177317831793180318131823183318431853186318731883189319031913192319331943195319631973198319932003201320232033204320532063207320832093210321132123213321432153216321732183219322032213222322332243225322632273228322932303231323232333234323532363237323832393240324132423243324432453246324732483249325032513252325332543255325632573258325932603261326232633264326532663267326832693270327132723273327432753276327732783279328032813282328332843285328632873288328932903291329232933294329532963297329832993300330133023303330433053306330733083309331033113312331333143315331633173318331933203321332233233324332533263327332833293330333133323333333433353336333733383339334033413342334333443345334633473348334933503351335233533354335533563357335833593360336133623363336433653366336733683369337033713372337333743375337633773378337933803381338233833384338533863387338833893390339133923393339433953396339733983399340034013402340334043405340634073408340934103411341234133414341534163417341834193420342134223423342434253426342734283429343034313432343334343435343634373438343934403441344234433444344534463447344834493450345134523453345434553456345734583459346034613462346334643465346634673468346934703471347234733474347534763477347834793480348134823483348434853486348734883489349034913492349334943495349634973498349935003501350235033504350535063507350835093510351135123513351435153516351735183519352035213522352335243525352635273528352935303531353235333534353535363537353835393540354135423543354435453546354735483549355035513552355335543555355635573558355935603561356235633564356535663567356835693570b";

uint8_t usart__request_command = 0;
uint8_t usart__respond_command = 0;

char cam_request_cartesian = 'Z';
char cam_request_radial = 'D';
char cam_request_close = 'q';


//typedef float pixel_t;
volatile uint8_t binarydata_flag = 0;

uint8_t binarydata_buffer0[CAM_BUF_SIZE];
//volatile uint8_t binarydata_buffer1[CAM_BUF_SIZE];
//volatile uint8_t binarydata_little[CAM_BUF_SIZE];
//
volatile float floatframe_buffer[CAM_FLOAT_SIZE];
//volatile float diff1_buffer[CAM_FLOAT_SIZE];
//volatile float diff2_buffer[CAM_FLOAT_SIZE];



static uint8_t msg[20]={0,};  // ethernet config printing


uint8_t cam_destip[4] = {192, 168, 178, 69};	//




uint16_t cam_destport = 50002;	//
uint16_t local_port = 49152;;	//


// global flags

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void usb_transmit_byte(uint8_t transmit_int);
void usb_transmit_int(int32_t transmit_int);
void usb_transmit_uint(uint32_t transmit_uint);
void usb_transmit_float(float transmit_float);
void usb_transmit_char(char transmit_char);  // test this
void usb_transmit_string(char *transmit_string);  // test this
void set_LED(bool status);

void cs_sel();

void cs_desel();

uint8_t spi_rb(void);

void spi_wb(uint8_t buf);

void spi_rb_burst(uint8_t *buf, uint16_t len);

void spi_wb_burst(uint8_t *buf, uint16_t len);



int32_t loopback_test(uint8_t sn, uint8_t* buf, uint16_t port);
/**
  * @brief  Read Data from camera vie W5500 on SPI.
  * @retval float
  */
int32_t get_camdata(void);
float canny_edge_mod(const float *floatbuf);
float canny_bad_solution(void);
// experimental stuff here:



/**
  * @brief  Low level check if float is negative value.
  * @retval 1 if x is negative, otherwise 0
  */
bool is_negative(float x) {  // fast check for negative float // only big endian?
//	usb_transmit_byte(signbit(x));
    return signbit(x);
}

void checkruntime(void (*fptr)())
{
	uint32_t starttime;
	uint32_t stoptime;
	uint32_t difftime;
	usb_transmit_string("Time: ");
	starttime = HAL_GetTick();  // milliseconds precision
	fptr();   // function pointer
	stoptime = HAL_GetTick();
	difftime = stoptime - starttime;
	usb_transmit_int(difftime);
	usb_transmit_string("\n\r");
}

// reordering from big endian to little endian
/**
  * @brief  Swapping uint32_t from big endian to little endian.
  * @retval uint32_t
  */
inline uint32_t SwapByteOrder_32(uint32_t a)  // ok
{
  return __builtin_bswap32(a);
}

/**
  * @brief  Swapping 4 bytes of uint8_t array from big endian to little endian.
  * @retval uint32_t
  */
//inline uint32_t byteswap2little(uint8_t *value)  // test it
inline uint32_t byteswap2little(uint8_t *value)  // test it
{
	uint32_t tmpint;
	memcpy(&tmpint, value, 4);
	return __builtin_bswap32(tmpint);
}

/**
  * @brief  Showing uint32_t bit representation of float value
  * @retval uint32_t
  */
inline uint32_t checkfloatbytes(float C) {
  union {
    uint32_t i;
    float f;
  } in;
  in.f = C;
//  out.i = SwapByteOrder_32(in.i);
  return in.i;
}

/**
  * @brief  Type punning access of 4 bytes as a 32 bit little endian float.
  * @retval float
  */
float pun2float(const uint8_t *buf) {
    // from https://blog.regehr.org/archives/959
  float num;
  memcpy(&num, buf, 4);
//    return __builtin_bswap32(num);
  return num;
}
/**
  * @brief  Swapping 4 bytes of uint8_t array and type punning access of 4 bytes as a 32 bit little endian float.
  * @retval float
  */
float swap2float(uint8_t *buf) {
// from https://blog.regehr.org/archives/959
	uint32_t tmpswap = byteswap2little(buf);
	float num;
	memcpy(&num, &tmpswap, 4);
	return num;
}


/**
  * @brief  Type punning access of 4 bytes as a 32 bit little endian uint32_t.
  * @retval uint32_t
  */
uint32_t pun2int(const uint8_t *buf) {
    // from https://blog.regehr.org/archives/959

  uint32_t num;
  memcpy(&num, buf, 4);
//  return __builtin_bswap32(num);  // swap endianness
  return num;
}
/**
  * @brief  Type punning access of 4 bytes as a 32 bit swapped bytes float.
  * @retval uint32_t
  */


void lcdtest(void)
{
	ILI9341_Fill_Screen(WHITE);
	ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
	ILI9341_Draw_Text("Slow draw by selecting", 10, 10, BLACK, 1, WHITE);
	ILI9341_Draw_Text("and adressing pixels", 10, 20, BLACK, 1, WHITE);
	HAL_Delay(2000);
	ILI9341_Fill_Screen(WHITE);

	uint32_t x, y;
	x = 0;
	y = 0;
	while (y < 200)
	{
	while ((x < 200) && (y < 200))
	{

		if(x % 2)
		{
			ILI9341_Draw_Pixel(x, y, BLACK);
		}

		x++;
	}

		y++;
		x = 0;
	}

	x = 0;
	y = 0;


	while (y < 200)
	{
	while ((x < 200) && (y < 200))
	{

		if(y % 2)
		{
			ILI9341_Draw_Pixel(x, y, BLACK);
		}

		x++;
	}

		y++;
		x = 0;
	}
	HAL_Delay(2000);
	usb_transmit_string("\r\nRepeating\r\n");
}


float accessfloatarray(uint8_t *buf, uint8_t floatposition)
{
	return swap2float((buf+4*floatposition));
}

void testfloatarray(void)
{
	float outfloat;
	//	int j = (3199-95);
		int j = 0;
		for(int i = j; i <j+10; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}

		for(int i = 50; i <60; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}

		for(int i = 3120; i <3136; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}

		for(int i = 3189; i <3200; i++)  // ok
		{
			usb_transmit_string("i = ");
			usb_transmit_int(i);  //
			usb_transmit_string(" - ");
			outfloat = floatframe_buffer[i];
			usb_transmit_float(outfloat);  //
			usb_transmit_string("\n\r");
		}
}


void arrayreshaping(const uint8_t *arrptr)
{
	for(size_t i = 0; i < 3200; i++)
	{
		floatframe_buffer[i] = swap2float(arrptr+376+4*i);  // ok
	}
}


// Ethernet init procedure

void IO_LIBRARY_Init(void) {
	uint8_t RXbufSize[] =	{ 16, 0, 0, 0, 0, 0, 0, 0, }; // socket buffer size is 0...16
	uint8_t TXbufSize[] =	{ 16, 0, 0, 0, 0, 0, 0, 0, }; // socket buffer size

	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
	reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);


	wizchip_init(TXbufSize, RXbufSize);
	wiz_NetInfo netInfo = { .mac =	{ 0x00, 0x08, 0xdc, 0xab, 0xaf, 0xfe },	// Mac address
							.ip =	{ 192, 168, 178, 1},		// IP address
							.sn =	{ 255, 255, 255, 0 },	// Subnet mask
							.gw =	{ 192,  168, 178, 69 } };	// Gateway address

	wizchip_setnetinfo(&netInfo);

	wizchip_getnetinfo(&netInfo);
	PRINT_NETINFO(netInfo);

	wiz_NetTimeout gWIZNETTIME = {.retry_cnt = 3,         //RCR = 3
	                               .time_100us = 2000};     //RTR = 2000
	wizchip_setinterruptmask(IK_SOCK_0);  // Enable interrupt on socket0
	ctlnetwork(CN_SET_TIMEOUT,(void*)&gWIZNETTIME);
	ctlnetwork(CN_GET_TIMEOUT, (void*)&gWIZNETTIME);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // start hardware stuff




  IO_LIBRARY_Init();
  HAL_UART_Receive_IT(&huart1, &usart__request_command, 1);
  ILI9341_Init();


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



	  uint32_t starttime;
	  uint32_t stoptime;
	  uint32_t difftime;

  set_LED(true);  // enable led
  while (1)
  {

	  lcdtest();

// 	  Exec time measurement template:
//	  uint32_t starttime;
//	  uint32_t stoptime;
//	  uint32_t difftime;

//	  starttime = HAL_GetTick();  // milliseconds precision
	  // do something here:
//	  stoptime = HAL_GetTick();
//	  difftime = stoptime - starttime;
//	  usb_transmit_string("Time: ");
//	  usb_transmit_int(difftime);
//	  usb_transmit_string("\n\r");
//	volatile uint8_t binarydata_buffer0[CAM_BUF_SIZE];
//	volatile uint8_t binarydata_buffer1[CAM_BUF_SIZE];
//	volatile float floatframe_buffer[CAM_FLOAT_SIZE];
//      HAL_Delay(300);
//
//
//	starttime = HAL_GetTick();  // milliseconds precision
//
//
//
//
//	  get_camdata();
//	  arrayreshaping(binarydata_buffer0);


//	  switch(binarydata_flag)
//	{
//		case 0:
//			arrayreshaping(binarydata_buffer1);
//			usb_transmit_string("\r\n reshape0 ok\r\n");
//			break;
//		 case 1:
//			 arrayreshaping(binarydata_buffer0);
//			 usb_transmit_string("\r\n reshape0 ok\r\n");
//			break;
//		default:
//			usb_transmit_string("\r\n Canny detector error\r\n");
//			break;
//	}
//	  canny_bad_solution();



//
//	  stoptime = HAL_GetTick();
//	  difftime = stoptime - starttime;
//	  usb_transmit_string("\n\rTime: ");
//	  usb_transmit_int(difftime);
//	  usb_transmit_string("\n\r");


//	  testmyarray(binarydata_flag);

		//test here:

		//test finish:


// print here:
//	  usb_transmit_string("\n\routput:\n\r");
//	  someint = checkfloatbytes(somefloat1);
//	  checkruntime(&get_camdata);
//	  usb_transmit_uint(someint);
//	  for (int i=0; i<8; i++)  // strlen() at runtime, sizeof() at compile
//	  {
//
//		  someint = pun2int(&inputarr32[(i*4)]);
//
//		  usb_transmit_string("\n\r");
//		  usb_transmit_string("i=");
//		  usb_transmit_uint(i);
//		  usb_transmit_string(" - ");
//
//
//
//
//		  usb_transmit_uint(someint);
//		  usb_transmit_string("\n\r");
//		  usb_transmit_string(" - ");
//		  outputint = byteswap2little(inputint);
//		  usb_transmit_uint(outputint);
//		  usb_transmit_string("\n\r");
//	  }

//	  usb_transmit_int((HAL_DMA_GetState(&hspi1)));  // is 0
//	  usb_transmit_string("endloop");


//	  SPI_DMATransmitCplt  // dont use
//	  usb_transmit_string("\n\loop end.\n\r");

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void usb_transmit_byte(uint8_t transmit_byte)  // ok but sprintf is dangerous
{
	HAL_UART_Transmit(&huart2, (uint8_t*) transmit_byte, 1, 100);  // alternativ strlen
}
void usb_transmit_int(int32_t transmit_int)  // ok but sprintf is dangerous
{
	uint8_t stringbuf[10];
	sprintf(stringbuf,  "%i", transmit_int);
	HAL_UART_Transmit(&huart2, (uint8_t*) stringbuf, strlen(stringbuf) , 100);
}
void usb_transmit_uint(uint32_t transmit_uint)  // ok but sprintf is dangerous
{
	uint8_t stringbuf[10];
	sprintf(stringbuf,  "%i", transmit_uint);

	HAL_UART_Transmit(&huart2, (uint8_t*) stringbuf, strlen(stringbuf) , 100);
}
void usb_transmit_char(char transmit_char)  // ok
{
	HAL_UART_Transmit(&huart2, &transmit_char, 1, 100);
}
void usb_transmit_string( char *transmit_string)  //  ok
{
	HAL_UART_Transmit(&huart2, (uint8_t*) transmit_string, strlen(transmit_string) , 100); // small bug.
}
void usb_transmit_float(float transmit_float)
{
	char fullstring[20];
	char *tmpSign = (transmit_float < 0) ? "-" : " ";
	float tmpVal = (transmit_float < 0) ? -transmit_float : transmit_float;
	int tmpInt1 = tmpVal;                  // Get the integer (9876543210).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.012).
	int tmpInt2 = (tmpFrac * 100);  // Turn into integer (12).
	sprintf (fullstring, "%s%d.%2d", tmpSign, tmpInt1, tmpInt2);
	HAL_UART_Transmit(&huart2, (uint8_t*) fullstring, strlen(fullstring), 100);
}

void cs_sel()
{
	HAL_GPIO_WritePin(ETHERNET_CS_GPIO_Port, ETHERNET_CS_Pin, GPIO_PIN_RESET); //CS LOW
}

void cs_desel()
{
	HAL_GPIO_WritePin(ETHERNET_CS_GPIO_Port, ETHERNET_CS_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void)
{
	uint8_t buf;
	HAL_SPI_Receive(&hspi3, &buf, 1, 0xFFFFFFFF);
	return buf;
}

void spi_wb(uint8_t buf)
{
	HAL_SPI_Transmit(&hspi3, &buf, 1, 0xFFFFFFFF);
}

void spi_rb_burst(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Receive(&hspi3, &buf, len, 0xFFFFFFFF);
}

void spi_wb_burst(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Transmit(&hspi3, &buf, len, 0xFFFFFFFF);
}

void set_LED(bool status)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, status);
}

//int32_t get_camdata(uint8_t *rxbuf0, uint8_t *rxbuf1) {
int32_t get_camdata(void)
{

	int32_t ret_send = 0;
	int32_t ret_sock = 0;
	int32_t ret_connect = 0;

	int32_t ret_rcv = 0;
	uint16_t size = 0;
	uint16_t sn = 0; // using only socket 0
	char testch = 'D';
	//   uint8_t testch = 0x44;
//   uint8_t testch = 9;

	while(!getPHYCFGR());  // phy link check

   switch(getSn_SR(DATA_SOCK))
   {
    	case SOCK_ESTABLISHED :
//    		usb_transmit_string("SocketEstablished.\r\n");


     // send data here:
    		ret_send = send(sn, &testch, 1);
    		// wait here between response
    	    HAL_Delay(10);  // delay for network reply needed




//    		do{
//    			usb_transmit_string("\n\r Waiting for data - ");
//    			size = getSn_RX_RSR(DATA_SOCK);
//    			usb_transmit_int(size);
//    			usb_transmit_string("\n\r");
//    		} while (size == 0);
//    		usb_transmit_string("\n\r received - ");
//    		usb_transmit_int(size);
//    		usb_transmit_string("\n\r");
    	     // receive data here:

// bug. wait for something in register.  // received size is not granted

    		if((size = getSn_RX_RSR(DATA_SOCK)) > 0){ // If data in rx buffer. maybe bug. wait for data in loop?
    			ret_rcv = recv(DATA_SOCK,binarydata_buffer0, CAM_BUF_SIZE);

    			// this should be a double buffer for DMA. it is not used since dma is disabled
//    			switch(binarydata_flag)
//				{
//					case 0:
//
//						ret_rcv = recv(DATA_SOCK,binarydata_buffer0, CAM_BUF_SIZE);
//
//						usb_transmit_string("binarydata_buffer0:");
//						usb_transmit_int(ret_rcv);
//						usb_transmit_string("\n\r");
//						binarydata_flag = 1;
//						break;
//					 case 1:
//
//						ret_rcv = recv(DATA_SOCK,binarydata_buffer1, CAM_BUF_SIZE);
//
//						usb_transmit_string("binarydata_buffer1:");
//						usb_transmit_int(ret_rcv);
//						usb_transmit_string("\n\r");
//						binarydata_flag = 0;
//						break;
//					default:
//						usb_transmit_string("/r/n Transmission error\r\n");
//						break;
//				}

				if(ret_rcv == CAM_BUF_SIZE)
				{

					usb_transmit_string("RX OK.\n\r");  // set busy state, wait for RX buffer
//					if(ret==SOCK_BUSY){
//						usb_transmit_string("SOCK_BUSY\n\r");
//						return 0;  // still waiting
//					}

				} else {
					usb_transmit_string("RX Err:\n\r");
					usb_transmit_int(ret_rcv);  //
					usb_transmit_string("\n\r");
					// hier timeout einbauen, da infinite loop in SOCK_ESTABLISHED
				}
    		} else {
    			usb_transmit_string("/r/n Transmission error \r\n");
				usb_transmit_int(size);
    		}
    		break;
   		case SOCK_CLOSE_WAIT :
			if((ret_connect=disconnect(DATA_SOCK)) != SOCK_OK) return ret_connect;
			#ifdef DEBUG_MODE:
			usb_transmit_string("CloseOK\r\n");
			#endif
   			break;
   		case SOCK_CLOSED :
			#ifdef DEBUG_MODE:
   			usb_transmit_string("SOCK_CLOSED\r\n");
			#endif
			if((ret_sock=socket(DATA_SOCK, Sn_MR_TCP, local_port, 0x0)) != DATA_SOCK)
			{
				usb_transmit_string("bug?\r\n");
				usb_transmit_int(ret_connect);
				close(DATA_SOCK);
				return ret_sock;
			}
			local_port++;  // avoid reusing local port
			if(local_port > 65000){
				local_port = 49152;
			}

   			break;

   		case SOCK_INIT :
			#ifdef DEBUG_MODE:
   			usb_transmit_string("SOCK_INIT\r\n");
			#endif

			while((ret_connect = connect(DATA_SOCK, cam_destip, cam_destport)) != SOCK_OK)  // -4 ,-13 timeout zu kurz?
			{
				#ifdef DEBUG_MODE
				HAL_Delay(10);
				usb_transmit_string("Connect error return:");
				usb_transmit_int(ret_connect);
				usb_transmit_string("r\n");
				usb_transmit_string("Reconnecting.\r\n");
				#endif
				return ret_connect;
			}
//			if((ret_connect = connect(DATA_SOCK, cam_destip, cam_destport)) != SOCK_OK)  // -4 ,-13 timeout zu kurz?
//			{
//				usb_transmit_string("Connect error return:\r\n");
//				usb_transmit_int(ret_connect);
//				usb_transmit_string("\r\n");
//				return ret_connect;
//			}
   			break;
   		default :
   			break;
   }
   return 0;  // finished
}





//raw stuff here:
//volatile uint8_t binarydata_buffer0[CAM_BUF_SIZE];
//volatile uint8_t binarydata_buffer1[CAM_BUF_SIZE];
//volatile float floatframe_buffer[CAM_FLOAT_SIZE];
//volatile float diff1_buffer[CAM_FLOAT_SIZE];  // 3200
//volatile float diff2_buffer[CAM_FLOAT_SIZE];  // 3200

// original returning edge array. mod version shall return edge yes/no?
float canny_edge_mod(const float *floatbuf)
{

	// detection hysteresis min, max
	const float tmin = 0.5;
	const float tmax = 1.0;
	//gauss filter sigma parameter
//	const float sigma = 1;  // not used. only needed for gauss filter

	// matrix shape

    const int nx = 64;
    const int ny = 50;

//    int MAX_BRIGHTNESS = 255;  // was
//    uint MAX_BRIGHTNESS = 255;  // max float brightness

    // some useful error checking


    // memory allocation
	float *in = calloc(nx * ny * sizeof(float), sizeof(float));
    float *G = calloc(nx * ny * sizeof(float), sizeof(float));
    float *after_Gx = calloc(nx * ny * sizeof(float), sizeof(float));
    float *after_Gy = calloc(nx * ny * sizeof(float), sizeof(float));
    float *nms = calloc(nx * ny * sizeof(float), sizeof(float));
    float *out = malloc(nx * ny * sizeof(float));


    	in = floatbuf;

//	some useless error checking
//    if (G == NULL || after_Gx == NULL || after_Gy == NULL ||
//        nms == NULL || out == NULL) {
//        fprintf(stderr, "canny_edge_detection:"
//                " Failed memory allocation(s).\n");
//        exit(1);  // check if error occured
//    }



//    gaussian_filter(in, out, nx, ny, sigma);
	const float Gx[] = {-1, 0, 1,
						-2, 0, 2,
						-1, 0, 1};

    // sobel operator for convolution
//    const float Gy[] = { 1, 2, 1,
//                         0, 0, 0,
//                        -1,-2,-1};
    const float Gy[] = { 1, 1, 1,
                         0, 0, 0,
                        -1,-1,-1};

//    convolution(in, after_Gy, Gy, nx, ny, 3, false);
// this is convolution() copypasted:
    	const int kn = 3;  // kernel size
        const int khalf = kn / 2;  // half of kernel. "center" of Gy matrix



//        float min = FLT_MAX, max = -FLT_MAX;  // min and max float values for normalization

//        if (normalize)  // if float value is overfloating
//            for (int m = khalf; m < nx - khalf; m++)
//                for (int n = khalf; n < ny - khalf; n++) {
//                    float pixel = 0.0;
//                    size_t c = 0;
//                    for (int j = -khalf; j <= khalf; j++)
//                        for (int i = -khalf; i <= khalf; i++) {
//                            pixel += in[(n - j) * nx + m - i] * kernel[c];
//                            c++;
//                        }
//                    if (pixel < min)
//                        min = pixel;
//                    if (pixel > max)
//                        max = pixel;
//                    }
        // iterating over input_array from "center"
        // convolution of input matrix and sobel operator
        // !bug - probably wrong array shape

        for (int m = khalf; m < nx - khalf; m++)
        {
            for (int n = khalf; n < ny - khalf; n++)
            {
                float pixel = 0.0;
                size_t c = 0;
                for (int j = -khalf; j <= khalf; j++)
                {
                    for (int i = -khalf; i <= khalf; i++)
                    {
                    	// multiply input_array with kernel matrix
                        pixel += in[(n - j) * nx + m - i] * Gy[c];
                        c++;
                    }

                    //
                    // result of convolution is a sobel operator matrix
                    after_Gy[n * nx + m] = pixel;
                }
            }
    	}


//    calculate intensity gradient
        // not used since gradient calculation is only made in y dir
//    for (int i = 1; i < nx - 1; i++)
//        for (int j = 1; j < ny - 1; j++) {
//            const int c = i + nx * j;
//            // G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
//            G[c] = (pixel_t)hypot(after_Gx[c], after_Gy[c]);
//        }

    // magnitude calculation disabled
    // Non-maximum suppression, straightforward implementation.

    // calculate surrounding
    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            const int nn = c - nx;
            const int ss = c + nx;
            const int ww = c + 1;
            const int ee = c - 1;
            const int nw = nn + 1;
            const int ne = nn - 1;
            const int sw = ss + 1;
            const int se = ss - 1;

//            const float dir = (float)(fmod(atan2(after_Gy[c], after_Gx[c]) + M_PI, M_PI) / M_PI) * 8;
            // calculate intensity vector direction. we use only Gy direction, so this one is not needed
            // if values on same axis differ too much, then nms[c] = G[c], else nms[c] = 0,
            //
            G = after_Gy;
            const float dir = (fmodf(after_Gy[c] + M_PI, M_PI) / M_PI) * 8.0f ;
            if (((dir <= 1 || dir > 7) && G[c] > G[ee] &&
                 G[c] > G[ww]) || // 0 deg
                ((dir > 1 && dir <= 3) && G[c] > G[nw] &&
                 G[c] > G[se]) || // 45 deg
                ((dir > 3 && dir <= 5) && G[c] > G[nn] &&
                 G[c] > G[ss]) || // 90 deg
                ((dir > 5 && dir <= 7) && G[c] > G[ne] &&
                 G[c] > G[sw]))   // 135 deg
                nms[c] = G[c];
            else
                nms[c] = 0;
        }

    // Reuse array
        // used as a stack. nx*ny/2 elements should be enough.
//        int *edges = (int*) after_Gy;
//
//        // filling *out and *edges arrays with zeros for reuse
//        memset(out, 0, sizeof(float) * nx * ny);
//        memset(edges, 0, sizeof(float) * nx * ny);
//
//        // Tracing edges with hysteresis . Non-recursive implementation.
//        size_t c = 1;
//        for (int j = 1; j < ny - 1; j++)
//        {
//        	for (int i = 1; i < nx - 1; i++)
//            {
//            	// if nms is over thresold and out is zero(why is this checked?) -> edge detected
//                if (nms[c] >= tmax && out[c] == 0)
//                {
//                    out[c] = FLT_MAX;
//
//                    int nedges = 1;
//                    edges[0] = c;  //c=1
//
//                    do {
//                        nedges--;  // nedges = 0
//                        const int t = edges[nedges];  // t =
//
//                        int nbs[8]; // neighbours
//                        nbs[0] = t - nx;     // nn
//                        nbs[1] = t + nx;     // ss
//                        nbs[2] = t + 1;      // ww
//                        nbs[3] = t - 1;      // ee
//                        nbs[4] = nbs[0] + 1; // nw
//                        nbs[5] = nbs[0] - 1; // ne
//                        nbs[6] = nbs[1] + 1; // sw
//                        nbs[7] = nbs[1] - 1; // se
//
//                        // iterating neighbors.
//                        for (int k = 0; k < 8; k++)
//                        {
//                        	//if neighbors are above tmin -> edge detected
//                        	if (nms[nbs[k]] >= tmin && out[nbs[k]] == 0)
//                            {
//                                out[nbs[k]] = FLT_MAX;
//                                edges[nedges] = nbs[k];
//                                nedges++;
//                                set_LED(true);  // enable led
//                                usb_transmit_string("\r\nEDGE!\r\n");
//                            } else {
////                            	set_LED(false);  // disable led
//                            }
//                        }
//
//                        //do iterate until end of
//                    } while (nedges > 0);
//                } else {
////                	set_LED(false);  // disable led
//                }
//                c++;
//            }
//        }

        for(size_t i = 1; i < 1000; i++)
        {


//        	if((after_Gy[i] -after_Gy[i-1]) >= tmin &&
//        			(after_Gy[i] -after_Gy[i-1]) <= tmax)
        	if((after_Gy[i] -after_Gy[i-1]) >= tmin )
        	{
//        		usb_transmit_string("\r\i - ");
//        		usb_transmit_uint(i);
//        		usb_transmit_string(" - ");
//
//				usb_transmit_float(after_Gy[i]);
//				usb_transmit_string("\r\n");
//        		usb_transmit_string("\r\nEDGE!\r\n");

        		set_LED(true);
//        		HAL_Delay(100);
        	}else {
        		set_LED(false);
        	}

        }
        // free reserved memory
        free(after_Gx);
        free(after_Gy);
        free(G);
        free(nms);

//        return out;
}



void convolution(const float *in, float *out, const float *kernel,
                 const int nx, const int ny, const int kn,
                 const bool normalize)
{
//	uint8_t MAX_BRIGHTNESS = 255;  // not used
    const int khalf = kn / 2;  // half of kernel
    float min = FLT_MAX, max = -FLT_MAX;  // min and max float values
    /*
    if (normalize)  // if float value is overfloating
    {


        for (int m = khalf; m < nx - khalf; m++)
        {
            for (int n = khalf; n < ny - khalf; n++) {
                float pixel = 0.0;
                size_t c = 0;
                for (int j = -khalf; j <= khalf; j++)
                {

                    for (int i = -khalf; i <= khalf; i++) {
                        pixel += in[(n - j) * nx + m - i] * kernel[c];
                        c++;
                    }
                if (pixel < min)
                    min = pixel;
                if (pixel > max)
                    max = pixel;
                }
            }
		}
	}
	*/
    for (int m = khalf; m < nx - khalf; m++)
    {
        for (int n = khalf; n < ny - khalf; n++)
        {
            float pixel = 0.0;
            size_t c = 0;
            for (int j = -khalf; j <= khalf; j++)
            {
                for (int i = -khalf; i <= khalf; i++)
                {
                    pixel += in[(n - j) * nx + m - i] * kernel[c];
                    c++;
                }
               out[n * nx + m] = (float)pixel;
            }
        }
	}
}

/*
 * gaussianFilter:
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 * determine size of kernel (odd #)
 * 0.0 <= sigma < 0.5 : 3
 * 0.5 <= sigma < 1.0 : 5
 * 1.0 <= sigma < 1.5 : 7
 * 1.5 <= sigma < 2.0 : 9
 * 2.0 <= sigma < 2.5 : 11
 * 2.5 <= sigma < 3.0 : 13 ...
 * kernelSize = 2 * int(2*sigma) + 3;
 */
void gaussian_filter(const float *in, float *out,
                     const int nx, const int ny, const float sigma)
{
    const int n = 2 * (int)(2 * sigma) + 3;
    const float mean = floorf(n / 2.0f);
    float kernel[n * n]; // variable length array

//    fprintf(stderr, "gaussian_filter: kernel size %d, sigma=%g\n",
//            n, sigma);
    size_t c = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            kernel[c] = exp(-0.5 * (pow((i - mean) / sigma, 2.0) +
                                    pow((j - mean) / sigma, 2.0)))
                        / (2 * M_PI * sigma * sigma);
            c++;
        }

    convolution(in, out, kernel, nx, ny, n, true);
}





float canny_bad_solution(void)
{

	// detection hysteresis min, max
	const float tmin = 0.5;
	const float tmax = 1.0;
	//gauss filter sigma parameter
//	const float sigma = 1;  // not used. only needed for gauss filter

	// matrix shape

    const int nx = 64;
    const int ny = 50;

//    int MAX_BRIGHTNESS = 255;  // was
//    uint MAX_BRIGHTNESS = 255;  // max float brightness

    // some useful error checking
//    float after_Gy;

    // memory allocation in heap
//	float *in = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *G = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *after_Gx = calloc(nx * ny * sizeof(float), sizeof(float));
//      float *after_Gy = calloc(3200, sizeof(float));
//    float *nms = calloc(nx * ny * sizeof(float), sizeof(float));
//    float *out = malloc(nx * ny * sizeof(float));

	const float *in = &floatframe_buffer;

// memory allocation in stack
//    float G[nx * ny];
//    float after_Gx[nx * ny];
    float after_Gy[nx * ny];
//    float after_Gy[3200];
//    float nms[nx * ny];
//    float out[nx * ny];




//    size_t isize = sizeof(out);
//    usb_transmit_uint(isize);
//    HAL_Delay(10000);
//    	in = &floatbuf;

    // sobel operator for convolution
//    gaussian_filter(in, out, nx, ny, sigma);


// conventional intensity gradient calculation
	const int32_t khalf = 1;
	const int32_t kn = 3;

	// bug? memory leak? wrong values for array? no dynamic memory alloc on stm32?
	const float Gx[] = {-1.0, 0.0, 1.0,
						-1.0, 0.0, 1.0,
						-1.0, 0.0, 1.0};

    const float Gy[] = { 1.0, 1.0, 1.0,
                         0.0, 0.0, 0.0,
                        -1.0,-1.0,-1.0};

//    float testfloat = 0.0;
//    for(int i = 0; i <3200; i++)  // ok
//	{
//		after_Gy[i] = testfloat;
//	}
	int outint;


	size_t row;
	size_t column;
	size_t pixel_position = row * nx + column;



        for (int m = khalf; m < nx - khalf; m++)  // m from 1 to 63
        {
            for (int n = khalf; n < ny - khalf; n++)  // n from 1 to 49
            {
                float pixel = 0.0;
                float tmpflaot = 0.0;
                size_t c = 0;
                for (int j = -khalf; j <= khalf; j++)  // j from -1 to 1
                {
                    for (int i = -khalf; i <= khalf; i++)  // i from -1 to 1
                    {
                    	// multiply input_array with kernel matrix

//                    	outint = ((n - j) * nx + m - i);
//            			usb_transmit_string("(n - j) * nx + m - i = ");
//            			usb_transmit_int(outint);  //
//            			usb_transmit_string(" - ");
//            			usb_transmit_float(pixel);  //
//            			usb_transmit_string("\n\r");
//            			HAL_Delay(300);

                        pixel += in[(n - j) * nx + m - i] * Gy[c];  // bug here> access in[i]
                        c++;
//                        if(pixel  )



                    }
                    //
                    // result of convolution is a gradient matrix
                    after_Gy[n * nx + m] = pixel;
                }
            }
    	}
//
//        for (int m = khalf; m < nx - khalf; m++)
//        {
//        	{
//                for (int n = khalf; n < ny - khalf; n++)
//                {
//                    float pixel = 0.0;
//                    size_t c = 0;
//                    for (int j = -khalf; j <= khalf; j++)
//                    {
//                        for (int i = -khalf; i <= khalf; i++)
//                        {
//                            pixel += in[(n - j) * nx + m - i] * Gx[c];
//                            c++;
//                        }
//                        after_Gx[n * nx + m] = pixel;
//                    }
//                }
//        	}
//        }
//		for (int i = 1; i < nx - 1; i++)
//		{
//			for (int j = 1; j < ny - 1; j++) {
//			 size_t c = i + nx * j; // i = column spalte, nx*j = row zeile
////			 G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
//			 G[c] = hypot(after_Gx[c], after_Gy[c]);
//			}
//		}



//	usb_transmit_string("\n\roverwritten\n\r");

    	float outfloat = 0.0;
//    	int outint;
//    	//	int j = (3199-95);
    		int j = 300;
    		for(int i = j; i <j+90; i++)  // ok
    		{
    			usb_transmit_string("i = ");
    			usb_transmit_int(i);  //
    			usb_transmit_string(" - ");
    			outfloat = after_Gy[i];
    			usb_transmit_float(outfloat);  //
//    			usb_transmit_int(outint);  //

    			usb_transmit_string("\n\r");
    		}
    		HAL_Delay(300);
//    		testfloatarray();
//    		for(int i = 50; i <60; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = floatframe_buffer[i];
//    			usb_transmit_float(outfloat);  //
//    			usb_transmit_string("\n\r");
//    		}
//
//    		for(int i = 3120; i <3136; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = floatframe_buffer[i];
//    			usb_transmit_float(outfloat);  //
//    			usb_transmit_string("\n\r");
//    		}
//
//    		for(int i = 3189; i <3200; i++)  // ok
//    		{
//    			usb_transmit_string("i = ");
//    			usb_transmit_int(i);  //
//    			usb_transmit_string(" - ");
//    			outfloat = floatframe_buffer[i];
//    			usb_transmit_float(outfloat);  //
//    			usb_transmit_string("\n\r");
//    		}
//        		set_LED(false);
        // free reserved memory
//        free(after_Gx);
//        free(after_Gy);
//        free(G);
//        free(nms);

//        return out;
    		return 0;
}



/*
 * Links:
 * http://en.wikipedia.org/wiki/Canny_edge_detector
 * http://www.tomgibara.com/computer-vision/CannyEdgeDetector.java
 * http://fourier.eng.hmc.edu/e161/lectures/canny/node1.html
 * http://www.songho.ca/dsp/cannyedge/cannyedge.html
 * https://medium.com/@nikatsanka/comparing-edge-detection-methods-638a2919476e
 * Note: T1 and T2 are lower and upper thresholds.
 */

/* original canny detection
pixel_t *canny_edge_detection_original(const pixel_t *in,
                              const bitmap_info_header_t *bmp_ih,
                              const int tmin, const int tmax,
                              const float sigma)
{
    const int nx = 64;
    const int ny = 50;

    pixel_t *G = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *after_Gx = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *after_Gy = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *nms = calloc(nx * ny * sizeof(pixel_t), 1);
    pixel_t *out = malloc(bmp_ih->bmp_bytesz * sizeof(pixel_t));

    if (G == NULL || after_Gx == NULL || after_Gy == NULL ||
        nms == NULL || out == NULL) {
        fprintf(stderr, "canny_edge_detection:"
                " Failed memory allocation(s).\n");
        exit(1);  // check if error occured
    }

    gaussian_filter(in, out, nx, ny, sigma);

    const float Gx[] = {-1, 0, 1,
                        -2, 0, 2,
                        -1, 0, 1};

    convolution(out, after_Gx, Gx, nx, ny, 3, false);

    const float Gy[] = { 1, 2, 1,
                         0, 0, 0,
                        -1,-2,-1};

    convolution(out, after_Gy, Gy, nx, ny, 3, false);

    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            // G[c] = abs(after_Gx[c]) + abs(after_Gy[c]);
            G[c] = (pixel_t)hypot(after_Gx[c], after_Gy[c]);
        }

    // Non-maximum suppression, straightforward implementation.
    for (int i = 1; i < nx - 1; i++)
        for (int j = 1; j < ny - 1; j++) {
            const int c = i + nx * j;
            const int nn = c - nx;
            const int ss = c + nx;
            const int ww = c + 1;
            const int ee = c - 1;
            const int nw = nn + 1;
            const int ne = nn - 1;
            const int sw = ss + 1;
            const int se = ss - 1;

            // calculate intensity gradient vector
            const float dir = (float)(fmod(atan2(after_Gy[c],
                                                 after_Gx[c]) + M_PI,
                                           M_PI) / M_PI) * 8;
            if (((dir <= 1 || dir > 7) && G[c] > G[ee] &&
                 G[c] > G[ww]) || // 0 deg
                ((dir > 1 && dir <= 3) && G[c] > G[nw] &&
                 G[c] > G[se]) || // 45 deg
                ((dir > 3 && dir <= 5) && G[c] > G[nn] &&
                 G[c] > G[ss]) || // 90 deg
                ((dir > 5 && dir <= 7) && G[c] > G[ne] &&
                 G[c] > G[sw]))   // 135 deg
                nms[c] = G[c];
            else
                nms[c] = 0;
        }

    // Reuse array
    // used as a stack. nx*ny/2 elements should be enough.
    int *edges = (int*) after_Gy;
    memset(out, 0, sizeof(pixel_t) * nx * ny);
    memset(edges, 0, sizeof(pixel_t) * nx * ny);

    // Tracing edges with hysteresis . Non-recursive implementation.
    size_t c = 1;
    for (int j = 1; j < ny - 1; j++)
        for (int i = 1; i < nx - 1; i++) {
            if (nms[c] >= tmax && out[c] == 0) { // trace edges
                out[c] = MAX_BRIGHTNESS;


                int nedges = 1;
                edges[0] = c;  //c=1

                do {
                    nedges--;  // nedges = 0
                    const int t = edges[nedges];  // t =

                    int nbs[8]; // neighbours
                    nbs[0] = t - nx;     // nn
                    nbs[1] = t + nx;     // ss
                    nbs[2] = t + 1;      // ww
                    nbs[3] = t - 1;      // ee
                    nbs[4] = nbs[0] + 1; // nw
                    nbs[5] = nbs[0] - 1; // ne
                    nbs[6] = nbs[1] + 1; // sw
                    nbs[7] = nbs[1] - 1; // se

                    for (int k = 0; k < 8; k++)
                        if (nms[nbs[k]] >= tmin && out[nbs[k]] == 0) {
                            out[nbs[k]] = MAX_BRIGHTNESS;
                            edges[nedges] = nbs[k];
                            nedges++;
                        }
                } while (nedges > 0);
            }
            c++;
        }

    free(after_Gx);
    free(after_Gy);
    free(G);
    free(nms);

    return out;
}
*/

//	free(after_Gx);
//    free(after_Gy);
//    free(G);
//    free(nms);

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	#ifdef DEBUG
	  asm("BKPT #0");
	#else
	  while(1)
	  {
		HAL_Delay(250);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }
	#endif

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
