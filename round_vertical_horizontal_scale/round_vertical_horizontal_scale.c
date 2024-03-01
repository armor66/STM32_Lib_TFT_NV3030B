/*

  ******************************************************************************
  * @file 			( фаил ):   round_vertical_horizontal_scale.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):	author: Golinskiy Konstantin	e-mail: golinskiy.konstantin@gmail.com
  ******************************************************************************
  
*/

/* Includes ----------------------------------------------------------*/
#include "round_vertical_horizontal_scale.h"

#include "math.h"
#include "NV3030B.h"
#include "stdio.h"


//==================================================================
//------------------------------------------------------------------
// функция инициализации круглой шкалы
void scale_round_init(scale_round_type* round)
{
	const double rad = 0.01745;
	
	for(uint16_t i = 0; i < 360; i++ )
  {
    round->pointOuterX[i] = (round->radiusRound * cos(rad * i)) + round->offsetCoordX;
    round->pointOuterY[i] = (round->radiusRound * sin(rad * i)) + round->offsetCoordY;
		
		// число длина короткой линии
    round->pointIneerX[i] = ((round->radiusRound - round->lengthLineIneer) * cos(rad * i)) + round->offsetCoordX;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
    round->pointIneerY[i] = ((round->radiusRound - round->lengthLineIneer) * sin(rad * i)) + round->offsetCoordY;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина длинной линии
    round->pointLongX[i] = ((round->radiusRound - round->lengthLineLong) * cos(rad * i)) + round->offsetCoordX;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
    round->pointLongY[i] = ((round->radiusRound - round->lengthLineLong) * sin(rad * i)) + round->offsetCoordY;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина средней линии
    round->pointShortX[i] = ((round->radiusRound - round->lengthLineShotr) * cos(rad * i)) + round->offsetCoordX;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
    round->pointShortY[i] = ((round->radiusRound - round->lengthLineShotr) * sin(rad * i)) + round->offsetCoordY;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число растояние от линий до текста
    round->pointTextX[i] = ((round->radiusRound + round->lengthRoundText) * cos(rad * i)) + round->offsetCoordX;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
    round->pointTextY[i] = ((round->radiusRound + round->lengthRoundText) * sin(rad * i)) + round->offsetCoordY;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
  }
}
//------------------------------------------------------------------	

//------------------------------------------------------------------	
// функция прорисовки круглой шкалы
// первый парам: структура с настройками шкалы
// второй парам: угол от 0 до 359 или от-180 до 180
// третий парам: формат ANGLE_FORMAT_180 или ANGLE_FORMAT_360
// четвертый парам: шинина дисплея в пикселях
// пяты парам: высота дисплея в пикселях
void scale_round_draw(scale_round_type* round, int16_t angle, angle_format_type format, uint16_t width_display, uint16_t height_display )
{
		int16_t a = 0;
		
		for(int16_t i = 0; i < 360; i++)
		{
			
		if( ANGLE_FORMAT_180 == format ){
			if( angle < 0 ) { angle = 360 + angle; }
		}
		
		a = angle + (i);
	 
    if(a > 359){ a = (angle + (i)) - 360; }
	
		// (a==angle) 			-> риска и текст нулевой метки
		// (360-i==angle) 	-> риска и текст текущего значения
	 
		if(( (round->pointOuterX[a] >= round->coeffUnvisibleLX) && (round->pointOuterX[a] < width_display - round->coeffUnvisibleRX) ) && 
										( (round->pointOuterY[a] >= round->coeffUnvisibleUY ) && 
										(round->pointOuterY[a] < height_display - round->coeffUnvisibleDY) )){	 
											
				// длинная линия ( кратность )
				if( i % 10 == 0 ){
					NV3030B_DrawLine(round->pointOuterX[a], round->pointOuterY[a], round->pointLongX[a], round->pointLongY[a], round->colorForeground);

					char buff[6] = {0, };
					
					if( ANGLE_FORMAT_180 == format ){
						sprintf(buff, "%d", ( i ? ((360-i)<=180 ? (360-i) : (360 - (360-i)) * -1 ) : 0 ) ); // чтобы совпадала надпись градусов ( если отрицательные не нужно выводить удалить -1 )
					}
					else if( ANGLE_FORMAT_360 == format ){
						sprintf(buff, "%d", ( i ? 360-i : 0 )  ); // чтобы совпадала надпись градусов
					}
				
					NV3030B_print( round->pointTextX[a], round->pointTextY[a] - 5, round->colorText, round->colorBackground, 1, &Font_7x9, 1, buff  );	// -5 корректировка текста по оси Y чтобы шрифт был на ровне с линией
				}
				// средняя линия ( кратность )
				else if( i % 5 == 0){
					NV3030B_DrawLine(round->pointOuterX[a], round->pointOuterY[a], round->pointShortX[a], round->pointShortY[a], round->colorForeground);
				}
				// короткие линии
				else{
					NV3030B_DrawLine(round->pointOuterX[a], round->pointOuterY[a], round->pointIneerX[a], round->pointIneerY[a], round->colorForeground);
				}
		}
	}
}
//------------------------------------------------------------------
//=========================================================================




//=========================================================================
//------------------------------------------------------------------
// функция инициализации вертикальной шкалы
void scale_vertical_init(scale_vertical_type* vertical)
{
	//----------------------
	for(int16_t i = 0; i < 360 - (vertical->offsetStartZeroY/vertical->offsetCoeff); i++)
  {
    vertical->pointOuterX[i] = vertical->offsetCoordX;
		vertical->pointTotalY[i] = (i) + (vertical->offsetStartZeroY/vertical->offsetCoeff);
		
		// число длина короткой линии
    vertical->pointIneerX[i] = vertical->offsetCoordX + vertical->lengthLineIneer;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина длинной линии
    vertical->pointLongX[i] = vertical->offsetCoordX + vertical->lengthLineLong;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина средней линии
    vertical->pointShortX[i] = vertical->offsetCoordX + vertical->lengthLineShotr;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число растояние от линий до текста
    vertical->pointTextX[i] = vertical->offsetCoordX - vertical->lengthVerticalText;  	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
  }
	//----------------------
	
	//----------------------
	for(int16_t i = 359; i > 360 -(vertical->offsetStartZeroY/vertical->offsetCoeff); i--)
  {
    vertical->pointOuterX[i] = vertical->offsetCoordX;
    vertical->pointTotalY[i] = (i) - 360 + (vertical->offsetStartZeroY/vertical->offsetCoeff);
		
		// число длина короткой линии
    vertical->pointIneerX[i]= vertical->offsetCoordX + vertical->lengthLineIneer;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина длинной линии
    vertical->pointLongX[i]= vertical->offsetCoordX + vertical->lengthLineLong;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина средней линии
    vertical->pointShortX[i]= vertical->offsetCoordX + vertical->lengthLineShotr;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число растояние от линий до текста
    vertical->pointTextX[i]= vertical->offsetCoordX - vertical->lengthVerticalText;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
  }
}
//------------------------------------------------------------------

//------------------------------------------------------------------	
// функция прорисовки вертикальной шкалы
// первый парам: структура с настройками шкалы
// второй парам: угол от 0 до 359 или от-180 до 180
// третий парам: формат ANGLE_FORMAT_180 или ANGLE_FORMAT_360
// четвертый парам: шинина дисплея в пикселях
// пяты парам: высота дисплея в пикселях
void scale_vertical_draw(scale_vertical_type* vertical, int16_t angle, angle_format_type format, uint16_t width_display, uint16_t height_display )
{
		int16_t a = 0;
		
		for(int16_t i = 0; i < 360; i++)
		{
			
		if( ANGLE_FORMAT_180 == format ){
			if( angle < 0 ) { angle = 360 + angle; }
		}
		
		a = angle + (i);
	 
    if(a > 359){ a = (angle + (i)) - 360; }
	
		// (a==angle) 			-> риска и текст нулевой метки
		// (360-i==angle) 	-> риска и текст текущего значения
	 
		if(( (vertical->pointOuterX[a] >= vertical->coeffUnvisibleLX) && (vertical->pointOuterX[a] < width_display - vertical->coeffUnvisibleRX) ) && 
										( (vertical->pointTotalY[a] * vertical->offsetCoeff >= vertical->coeffUnvisibleUY ) && 
										(vertical->pointTotalY[a] * vertical->offsetCoeff < height_display - vertical->coeffUnvisibleDY) )){	 
											
				// длинная линия ( кратность )
				if( i % 10 == 0 ){
					NV3030B_DrawLine(vertical->pointOuterX[a], vertical->pointTotalY[a] * vertical->offsetCoeff, vertical->pointLongX[a], vertical->pointTotalY[a] * vertical->offsetCoeff, vertical->colorForeground);

					char buff[6] = {0, };
					
					if( ANGLE_FORMAT_180 == format ){
						sprintf(buff, "%4d", ( i ? ((360-i)<=180 ? (360-i) : (360 - (360-i)) * -1 ) : 0 ) ); // чтобы совпадала надпись градусов ( если отрицательные не нужно выводить удалить -1 )
					}
					else if( ANGLE_FORMAT_360 == format ){
						sprintf(buff, "%4d", ( i ? 360-i : 0 )  ); // чтобы совпадала надпись градусов
					}
				
					NV3030B_print( vertical->pointTextX[a], (vertical->pointTotalY[a] * vertical->offsetCoeff) - 5, vertical->colorText, vertical->colorBackground, 1, &Font_7x9, 1, buff  );	// -5 корректировка текста по оси Y чтобы шрифт был на ровне с линией
				}
				// средняя линия ( кратность )
				else if( i % 5 == 0){
					NV3030B_DrawLine(vertical->pointOuterX[a], vertical->pointTotalY[a] * vertical->offsetCoeff, vertical->pointShortX[a], vertical->pointTotalY[a] * vertical->offsetCoeff, vertical->colorForeground);
				}
				// короткие линии
				else{
					NV3030B_DrawLine(vertical->pointOuterX[a], vertical->pointTotalY[a] * vertical->offsetCoeff, vertical->pointIneerX[a], vertical->pointTotalY[a] * vertical->offsetCoeff, vertical->colorForeground);
				}
		}
	}
}
//------------------------------------------------------------------
//==================================================================





//==================================================================
//------------------------------------------------------------------
// функция инициализации горизонтальной шкалы
void scale_horizontal_init(scale_horizontal_type* horizontal)
{
	//----------------------
	for(int16_t i = 0; i < 360 - (horizontal->offsetStartZeroX/horizontal->offsetCoeff); i++)
  {
    horizontal->pointOuterY[i] = horizontal->offsetCoordY;
		horizontal->pointTotalX[i] = (i) + (horizontal->offsetStartZeroX/horizontal->offsetCoeff);
		
		// число длина короткой линии
    horizontal->pointIneerY[i] = horizontal->offsetCoordY - horizontal->lengthLineIneer; // ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина длинной линии
    horizontal->pointLongY[i] = horizontal->offsetCoordY - horizontal->lengthLineLong;		// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина средней линии
    horizontal->pointShortY[i] = horizontal->offsetCoordY - horizontal->lengthLineShotr;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число растояние от линий до текста
    horizontal->pointTextY[i] = horizontal->offsetCoordY + horizontal->lengthHorizontalText; 	// ( если нужно инвертировать шкалу меняем + на - или на оборот ) 
  }
	//----------------------
	
	//----------------------
	for(int16_t i = 359; i > 360 -(horizontal->offsetStartZeroX/horizontal->offsetCoeff); i--)
  {
    horizontal->pointOuterY[i] = horizontal->offsetCoordY;
    horizontal->pointTotalX[i] = (i) - 360 + (horizontal->offsetStartZeroX/horizontal->offsetCoeff);
		
		// число длина короткой линии
    horizontal->pointIneerY[i]= horizontal->offsetCoordY - horizontal->lengthLineIneer;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина длинной линии
    horizontal->pointLongY[i]= horizontal->offsetCoordY - horizontal->lengthLineLong;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число длина средней линии
    horizontal->pointShortY[i]= horizontal->offsetCoordY - horizontal->lengthLineShotr;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )

		// число растояние от линий до текста
    horizontal->pointTextY[i]= horizontal->offsetCoordY + horizontal->lengthHorizontalText;	// ( если нужно инвертировать шкалу меняем + на - или на оборот )
  }
}
//------------------------------------------------------------------

//------------------------------------------------------------------	
// функция прорисовки горизонтальной шкалы
// первый парам: структура с настройками шкалы
// второй парам: угол от 0 до 359 или от-180 до 180
// третий парам: формат ANGLE_FORMAT_180 или ANGLE_FORMAT_360
// четвертый парам: шинина дисплея в пикселях
// пяты парам: высота дисплея в пикселях
void scale_horizontal_draw(scale_horizontal_type* horizontal, int16_t angle, angle_format_type format, uint16_t width_display, uint16_t height_display )
{
		int16_t a = 0;
		
		for(int16_t i = 0; i < 360; i++)
		{
			
		if( ANGLE_FORMAT_180 == format ){
			if( angle < 0 ) { angle = 360 + angle; }
		}
		
		a = angle + (i);
	 
    if(a > 359){ a = (angle + (i)) - 360; }
	
		// (a==angle) 			-> риска и текст нулевой метки
		// (360-i==angle) 	-> риска и текст текущего значения
	 
		if(( (horizontal->pointTotalX[a] * horizontal->offsetCoeff >= horizontal->coeffUnvisibleLX) && (horizontal->pointTotalX[a] * horizontal->offsetCoeff < width_display - horizontal->coeffUnvisibleRX) ) && 
										( (horizontal->pointOuterY[a] >= horizontal->coeffUnvisibleUY ) && 
										(horizontal->pointOuterY[a] < height_display - horizontal->coeffUnvisibleDY) )){	 
											
				// длинная линия ( кратность )
				if( i % 10 == 0 ){
					NV3030B_DrawLine(horizontal->pointTotalX[a] * horizontal->offsetCoeff, horizontal->pointOuterY[a], horizontal->pointTotalX[a] * horizontal->offsetCoeff, horizontal->pointLongY[a], horizontal->colorForeground);

					char buff[6] = {0, };
					
					if( ANGLE_FORMAT_180 == format ){
						sprintf(buff, "%d", ( i ? ((360-i)<=180 ? (360-i) : (360 - (360-i)) * -1 ) : 0 ) ); // чтобы совпадала надпись градусов ( если отрицательные не нужно выводить удалить -1 )
					}
					else if( ANGLE_FORMAT_360 == format ){
						sprintf(buff, "%d", ( i ? 360-i : 0 )  ); // чтобы совпадала надпись градусов
					}
				
					NV3030B_print( (horizontal->pointTotalX[a] * horizontal->offsetCoeff) - 5, (horizontal->pointTextY[a]), horizontal->colorText, horizontal->colorBackground, 1, &Font_7x9, 1, buff  );	// -5 корректировка текста по оси Х чтобы шрифт был на ровне с линией
				}
				// средняя линия ( кратность )
				else if( i % 5 == 0){
					NV3030B_DrawLine(horizontal->pointTotalX[a] * horizontal->offsetCoeff, horizontal->pointOuterY[a], horizontal->pointTotalX[a] * horizontal->offsetCoeff, horizontal->pointShortY[a], horizontal->colorForeground);
				}
				// короткие линии
				else{
					NV3030B_DrawLine(horizontal->pointTotalX[a] * horizontal->offsetCoeff, horizontal->pointOuterY[a], horizontal->pointTotalX[a] * horizontal->offsetCoeff, horizontal->pointIneerY[a], horizontal->colorForeground);
				}
		}
	}
}
//------------------------------------------------------------------
//=========================================================================


//----------------------------------------------------------------------------------

/************************ (C) COPYRIGHT GKP *****END OF FILE****/
