<div align="center">

# ⚡ SDL Circuit Simulator

**شبیه‌ساز و ویرایشگر تعاملی مدارهای الکتریکی مبتنی بر C++20 و کتابخانه گرافیکی SDL2**

![C++](https://img.shields.io/badge/C%2B%2B-20-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![SDL2](https://img.shields.io/badge/SDL2-GUI-E34F26?style=for-the-badge&logo=libsdl&logoColor=white)
![CMake](https://img.shields.io/badge/CMake-3.29+-064F8C?style=for-the-badge&logo=cmake&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-2EA44F?style=for-the-badge)

</div>

---

## 📖 درباره پروژه

**SDL Circuit Simulator** یک ابزار دسکتاپ سبک و تعاملی برای طراحی، ویرایش و تحلیل مدارهای الکتریکی است. این برنامه با استفاده از **تحلیل گره اصلاح‌شده (Modified Nodal Analysis - MNA)** و روش حذف گاوسی، دستگاه معادلات مدار را تشکیل داده و ولتاژ گره‌ها و جریان عبوری از شاخه‌ها را محاسبه می‌کند.

---

## ✨ ویژگی‌ها و امکانات

* 🖥️ **محیط گرافیکی تعاملی (GUI):** نمایش شبکه‌ای از ۷۰ گره (`7 × 10`) با قابلیت جابه‌جایی گره‌ها به صورت **Drag & Drop**.
* 🔌 **پشتیبانی از المان‌های متنوع:** رسم شماتیک اختصاصی برای **مقاومت (Resistor)**، **خازن (Capacitor)**، **سلف (Inductor)**، **منبع ولتاژ (Voltage Source)** و **منبع جریان (Current Source)**.
* 📂 **بارگذاری مدار از فایل (Netlist Parser):** خواندن خودکار ساختار مدار از فایل متنی با استفاده از عبارات باقاعده (`Regex`).
* 🧮 **حلگر ماتریسی (MNA Solver):** محاسبه دقیق ولتاژ هر گره و جریان المان‌ها با مدیریت خطاهای مدار (عدم وجود گره زمین یا تعدد گره زمین).
* 🛠️ **ویرایش زنده در محیط برنامه:**
  * تعیین یا تغییر **گره زمین (Ground)**
  * نام‌گذاری اختصاصی گره‌ها با بررسی عدم تکرار نام
  * تغییر مقدار المان‌ها (`Value`) و حذف المان‌های انتخاب‌شده (`Delete`)

---

## 🧩 فرمت فایل ورودی مدار

برای بارگذاری مدار از طریق دکمه `Get File Address`، دستورات را به شکل زیر در یک فایل متنی ذخیره کنید:

```text
add <element_type> <name> <from_node> <to_node> <value>