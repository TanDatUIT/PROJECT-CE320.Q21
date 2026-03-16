#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"

// Khai báo prototype của hàm để main hiểu (nếu viết hàm ở dưới cùng)
void testSDCard();

void setup() {
    // 1. Khởi tạo Serial để xem kết quả trên Monitor
    Serial.begin(115200);
    delay(2000); // Đợi 2 giây để nguồn ổn định và Serial sẵn sàng
    
    Serial.println("--- KHỞI ĐỘNG DỰ ÁN ESP32 EDGE COMPUTING ---");

    // 2. Gọi hàm kiểm tra thẻ SD
    testSDCard();
}

void loop() {
    // Tạm thời để trống. 
    // Sau này đây sẽ là nơi Huy gọi luồng: Capture -> Fuzzy Edge -> Save SD.
}

// Nội dung hàm kiểm tra thẻ SD Huy đã viết
void testSDCard() {
    Serial.println("\n--- Đang kiểm tra thẻ nhớ SanDisk ---");

    // Khởi tạo SD_MMC ở chế độ 1-bit (true) để tránh xung đột đèn Flash (GPIO 4)
    if(!SD_MMC.begin("/sdcard", true)){
        Serial.println("❌ Lỗi: Không thể gắn (mount) thẻ SD.");
        return;
    }

    uint8_t cardType = SD_MMC.cardType();
    if(cardType == CARD_NONE){
        Serial.println("❌ Lỗi: Không tìm thấy thẻ nhớ.");
        return;
    }

    Serial.print("✅ Loại thẻ: ");
    if(cardType == CARD_SDHC) Serial.println("SDHC (Chuẩn chuẩn!)");
    else Serial.println("Khác");

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("📊 Dung lượng thẻ: %llu MB\n", cardSize);

    // Ghi thử file log cho thuật toán mờ
    File file = SD_MMC.open("/Hello_UIT.txt", FILE_WRITE);
    if(file){
        file.println("Dự án: Nhận diện cạnh bằng Logic mờ.");
        file.println("Huy UIT - Computer Engineering.");
        file.close();
        Serial.println("✅ Đã ghi file thành công.");
    }

    Serial.println("--- Kiểm tra hoàn tất! ---");
}