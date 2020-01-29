import processing.serial.*;

// =========== CONSTANTS ==================
final int     BLACK              = 0;
final int     WHITE              = 255;
final color[] GRADIENT           = new color[ ] { color( 0, 0, 0 ), color( 101, 50, 110 ), color( 191, 57, 81 ), color( 252, 172, 75 ), color( 255, 255, 255 ) };

final int     SKIN_COLS          = 12;
final int     SKIN_ROWS          = 12;
final int     SKIN_CELLS         = SKIN_COLS * SKIN_ROWS;

final int     SKIN_SKIP_ROWS     = 0;
final int     SKIN_BUFFER_OFFSET = SKIN_SKIP_ROWS * SKIN_COLS;

final float   PRESS_FACTOR       = .6; //0.5, sensibility
final int     MIN_RANGE          = 0;
final int     MAX_RANGE          = 600;

final int     CELL_W             = 50;
final int     CELL_H             = 50;

final int     DISPLAY_W          = CELL_W * SKIN_COLS;
final int     DISPLAY_H          = CELL_H * ( SKIN_ROWS - SKIN_SKIP_ROWS );

final int     SERIAL_PORT        = 0; //32
final int     SERIAL_RATE        = 2000000;

final char    SKIN_DATA_EOS      = '\n';
final char    SKIN_DATA_SEP      = ',';

// =========== VARIABLES ==================
Serial  skin;
int[ ]  skinBuffer;
String  skinData      = null;
boolean skinDataValid = false;
PImage  skinImage     = createImage( DISPLAY_W, DISPLAY_H, RGB );

// =========== FUNC =======================
void readSkinBuffer( ) {
  while( skin.available( ) > 0 ) {
     skinData = skin.readStringUntil( SKIN_DATA_EOS );
     if( skinData != null ) {
        skinBuffer    = int( split( skinData, SKIN_DATA_SEP ) );
        skinDataValid = skinBuffer.length == SKIN_CELLS;
     }
   }
}

double sigmoid( double x ) { return 1. / ( 1. + Math.exp( -x ) ); }

double scaleBetween(double unscaledNum, int minAllowed, int maxAllowed, int min, int max) {
  return (maxAllowed - minAllowed) * (unscaledNum - min) / (max - min) + minAllowed;
}

void drawSkinHeatMap( ) {
  noStroke( );
  background( GRADIENT[ 0 ] );
  for( int i = SKIN_BUFFER_OFFSET; i < SKIN_CELLS; i++ ) {
    int   id  =  i - SKIN_BUFFER_OFFSET;
    int   X   = ( id % SKIN_COLS ) * CELL_W;
    int   Y   = ( id / SKIN_COLS ) * CELL_H;
    if (skinBuffer[ i ] < MIN_RANGE){skinBuffer[ i ] = 0;}
    if (skinBuffer[ i ] > MAX_RANGE){skinBuffer[ i ] = 0;}
    float val = (float) sigmoid( skinBuffer[ i ] * PRESS_FACTOR - 8 );
    //float val = (float) scaleBetween(skinBuffer[ i ], 0, 100, MIN_RANGE, MAX_RANGE);
    float in  = Math.min( Math.max( val, 0. ), 1. ) * WHITE;
    color c   = computeColor( in );
    
    for( int y = Y; y < Y + CELL_H; y++ ) {
       for( int x = X; x < X + CELL_W; x++ ) {
         skinImage.pixels[ y * DISPLAY_W + x ] = c;  
       }
    }
  }
  skinImage.updatePixels( );
  image( skinImage, 0, 0 );
}

color computeColor( float value ) {
  float s_value = value * ( GRADIENT.length - 1 ) / 255;
  int   inf_idx = Math.max( ( int ) Math.floor( s_value ), 0 );
  float decimal = s_value - inf_idx;
  int   sup_idx = Math.min( ( int ) Math.ceil( s_value ), GRADIENT.length - 1 );
  float t       = decimal;
  return lerpColor( GRADIENT[ inf_idx ], GRADIENT[ sup_idx ], t );
}

// =========== SET UP =====================
void settings( ) { size( DISPLAY_W, DISPLAY_H ); }
void setup   ( ) { //printArray(Serial.list());
skin = new Serial( this, Serial.list( )[ SERIAL_PORT ], SERIAL_RATE ); }

// =========== LOOP =======================
void draw() {
  readSkinBuffer( );
  if( !skinDataValid ) return;
  drawSkinHeatMap( );
}
