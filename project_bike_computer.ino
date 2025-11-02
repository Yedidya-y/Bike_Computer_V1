// --- שלב 1: ייבוא ספריות והגדרות ---

#include <Wire.h> // ייבוא ספריית "Wire", משמשת לתקשורת I2C (עם המסך)
#include <Adafruit_GFX.h> // ייבוא ספריית "GFX", ארגז כלים לציור צורות וטקסט
#include <Adafruit_SSD1306.h> // ייבוא הדרייבר הספציפי למסך שלנו (SSD1306)


// #define לטובת שינויים עדתידיים אם צריך
#define SCREEN_WIDTH 128 // קביעת "כינוי" לרוחב המסך בפיקסלים
#define SCREEN_HEIGHT 64 // קביעת "כינוי" לגובה המסך בפיקסלים

// פקודת יצירת "אובייקט" (ישות תוכנתית) של מסך:
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// 1. קורא לו 'display'
// 2. אומר לו את הרוחב והגובה
// 3. אומר לו להשתמש בספריית 'Wire' לתקשורת
// 4. אומר לו שאין לנו פין איפוס (Reset) מחובר

#define CRANK_SENSOR_PIN 34 // קביעת "כינוי" לפין 34, אליו נחבר את חיישן הפדל
#define WHEEL_SENSOR_PIN 35 // קביעת "כינוי" לפין 35, אליו נחבר את חיישן הגלגל


// --- שלב 2: משתנים גלובליים ---

/* (ההערות המעולות שלך על סוגי המשתנים)
נזכור שfloat זה משתנים צפים כלומר מספר עם נק עשרונית
נזכור שלונג מחיזק מספרי ענק שאינטיגר רגיל לא מסוגל
volatile משתנה בעל ערך לא קבוע שיכול להשתנות בכל רגע נתון
*/

// "מגירות" זיכרון למדידת זמן הפדל (חייב volatile כי הפסיקה משנה אותם)
volatile unsigned long lastCrankEventTime = 0; // שומר את זמן הקליק האחרון של הפדל
volatile unsigned long crankInterval = 0;      // שומר את הפרש הזמן בין שני קליקים בפדל

// "מגירות" זיכרון למדידת זמן הגלגל (חייב volatile)
volatile unsigned long lastWheelEventTime = 0; // שומר את זמן הקליק האחרון של הגלגל
volatile unsigned long wheelInterval = 0;      // שומר את הפרש הזמן בין שני קליקים בגלגל

// "מגירות" זיכרון לתוצאות החישוב (float כי הן מספרים עשרוניים)
float crankRPM = 0.0; // "מגירה" לשמירת חישוב סל"ד הפדל
float wheelRPM = 0.0; // "מגירה" לשמירת חישוב סל"ד הגלגל
float gearRatio = 0.0; // "מגירה" לשמירת חישוב יחס ההעברה

// --- שלב 3: פונקציות הפסיקה (ISRs) ---
// פונקציות מהירות שרצות אוטומטית כשהחיישן מזהה מגנט

void IRAM_ATTR onCrank() {
  // כאן נכתוב את הקוד שתופס את "הקליק" מהפדל
  // 1. קח את הזמן הנוכחי במיקרו-שניות
  unsigned long now = micros(); 
  
  // 2. חשב את הפרש הזמן מאז הפעם הקודמת
  crankInterval = now - lastCrankEventTime; 
  
  // 3. שמור את הזמן הנוכחי בתור "הפעם הקודמת"
  lastCrankEventTime = now;
}

void IRAM_ATTR onWheel() {
  // כאן נכתוב את הקוד שתופס את "הקליק" מהגלגל
  // 1. קח את הזמן הנוכחי במיקרו-שניות
  unsigned long now = micros();
  
  // 2. חשב את הפרש הזמן מאז הפעם הקודמת
  wheelInterval = now - lastWheelEventTime;
  
  // 3. שמור את הזמן הנוכחי בתור "הפעם הקודמת"
  lastWheelEventTime = now;

}

// --- שלב 4: פונקציית האתחול (setup) ---
// פונקציה שרצה פעם אחת בלבד, כשה-ESP32 נדלק

void setup() {
  // 1. אתחול המסך
  // הפקודה .begin מנסה "לדבר" עם המסך בכתובת 0x3C (הכתובת הסטנדרטית)
  // אם היא נכשלת (המסך לא מחובר נכון), היא מחזירה false
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    // אם נכשל, נתקע כאן לנצח. זה סימן לבדוק חיבורים.
    for (;;); 
  }

  // 2. ניקוי והצגת הודעת אתחול
  display.clearDisplay(); // נקה את כל מה שהיה על המסך
  display.setTextSize(1); // קבע גודל גופן קטן
  display.setTextColor(SSD1306_WHITE); // קבע צבע טקסט לבן
  display.setCursor(0, 0); // הזז את הסמן לפינה השמאלית העליונה
  display.println("System booting..."); // הדפס את ההודעה
  display.display(); // "דחוף" את כל מה שציירנו אל המסך
  delay(1000); // המתן שנייה כדי שנספיק לראות את ההודעה

  // 3. הגדרת פינים של חיישנים
  // קובע את הפינים כ-INPUT (קלט)
  // וה-PULLUP מפעיל נגד פנימי בתוך ה-ESP32
  // שגורם לפין להיות "גבוה" (3.3V) כברירת מחדל
  pinMode(CRANK_SENSOR_PIN, INPUT_PULLUP);
  pinMode(WHEEL_SENSOR_PIN, INPUT_PULLUP);

  // 4. חיבור הפסיקות (הפעלת ה"מלכודות")
  // אומר ל-ESP32:
  // "כאשר המתח בפין CRANK_SENSOR_PIN נופל (FALLING), תריץ את פונקציית onCrank"
  attachInterrupt(digitalPinToInterrupt(CRANK_SENSOR_PIN), onCrank, FALLING);
  // "כאשר המתח בפין WHEEL_SENSOR_PIN נופל (FALLING), תריץ את פונקציית onWheel"
  attachInterrupt(digitalPinToInterrupt(WHEEL_SENSOR_PIN), onWheel, FALLING);
}

// --- שלב 5: הלולאה הראשית (loop) ---
// פונקציה שרצה בלולאה אינסופית אחרי ש-setup מסתיים

void loop() {
  // כאן נעשה את החישובים, נצייר למסך, ונחכה רגע
  // --- 1. העתקה בטוחה של משתנים ---
  // נצטרך לקרוא את המשתנים שהפסיקות כותבות אליהם.
  // כדי למנוע מצב שאנחנו קוראים משתנה בדיוק כשהפסיקה כותבת עליו,
  // נעתיק אותם במהירות למשתנים מקומיים כשהפסיקות כבויות.
  
  unsigned long localCrankInterval;
  unsigned long localWheelInterval;
  unsigned long localLastCrankTime;
  unsigned long localLastWheelTime;

  noInterrupts(); // "אל תפריעו לי לשנייה" - כבה פסיקות
  localCrankInterval = crankInterval;
  localWheelInterval = wheelInterval;
  localLastCrankTime = lastCrankEventTime;
  localLastWheelTime = lastWheelEventTime;
  interrupts(); // "סיימתי, אפשר להפריע" - הפעל פסיקות בחזרה

  // --- 2. חישוב סל"ד (RPM) ---
  
  // בדיקת "פסק זמן" לפדל:
  // אם עברו יותר מ-2 שניות (2,000,000 מיקרו-שניות) מאז הקליק האחרון
  if (micros() - localLastCrankTime > 2000000) {
    crankRPM = 0.0; // נאפס את הסל"ד. הרוכב כנראה עצר.
  } else if (localCrankInterval > 0) { // אם יש לנו מדידה תקפה
    // חישוב: 60,000,000 (מיקרו-שניות בדקה) חלקי הזמן לסיבוב (במיקרו-שניות)
    crankRPM = 60000000.0 / localCrankInterval;
  }

  // בדיקת "פסק זמן" לגלגל:
  if (micros() - localLastWheelTime > 2000000) {
    wheelRPM = 0.0; // נאפס את סל"ד הגלגל
  } else if (localWheelInterval > 0) {
    wheelRPM = 60000000.0 / localWheelInterval;
  }
  
  // --- 3. חישוב יחס העברה ---
  
  // נחשב רק אם יש סל"ד פדל (כדי למנוע חלוקה באפס)
  if (crankRPM > 0 && wheelRPM > 0) {
    gearRatio = wheelRPM / crankRPM;
  } else {
    gearRatio = 0.0; // אם הפדל לא מסתובב, אין יחס
  }

  // --- 4. הצגת הנתונים על המסך ---
  
  display.clearDisplay(); // נקה את המסך לקראת הציור החדש
  display.setCursor(0, 0);  // החזר את הסמן לפינה השמאלית העליונה

  // הדפסת כל הנתונים שחישבנו:
  display.print("Crank RPM: ");
  display.println(crankRPM, 1); // ", 1" = הצג ספרה אחת אחרי הנקודה

  display.print("Wheel RPM: ");
  display.println(wheelRPM, 1);

  display.print("Gear Ratio: ");
  display.println(gearRatio, 2); // ", 2" = הצג שתי ספרות אחרי הנקודה

  display.display(); // "דחוף" את הציור החדש למסך הפיזי
  
  // --- 5. המתנה קצרה ---
  delay(100); // המתן 100 מילי-שניות (0.1 שנייה)
                // זה יוצר קצב רענון של 10 פריימים בשנייה.
                // זה מונע מהמסך להבהב מהר מדי ונעים לעין.
}