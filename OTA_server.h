#include <SD.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <EEPROM.h>
#include "ESP_Mini.h"
#include "OTA_html.h"
bool downloading_file=false;
const char* host = "esp32";
extern const char ESP_type[16];
WebServer server(80);

//SD Card webinterface download section
String webpage = ""; //String to save the html code
String loginIndex = "";
void append_page_header(){
  webpage = html_header;
}

//Saves repeating many lines of code for HTML page footers
void append_page_footer()
{ 
  webpage += F("</body></html>");
}

//SendHTML_Header
void SendHTML_Header()
{
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate"); 
  server.sendHeader("Pragma", "no-cache"); 
  server.sendHeader("Expires", "-1"); 
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
  server.send(200, "text/html", ""); //Empty content inhibits Content-length header so we have to close the socket ourselves. 
  append_page_header();
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Content
void SendHTML_Content()
{
  server.sendContent(webpage);
  webpage = "";
}

//SendHTML_Stop
void SendHTML_Stop()
{
  server.sendContent("");
  server.client().stop(); //Stop is needed because no content length was sent
}

//ReportSDNotPresent
void ReportSDNotPresent()
{
  SendHTML_Header();
  webpage += F("<h3>No SD Card present</h3>"); 
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportFileNotPresent
void ReportFileNotPresent(String target)
{
  SendHTML_Header();
  webpage += F("<h3>File does not exist</h3>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//File size conversion
String file_size(int bytes)
{
  String fsize = "";
  if (bytes < 1024)                 fsize = String(bytes)+" B";
  else if(bytes < (1024*1024))      fsize = String(bytes/1024.0,3)+" KB";
  else if(bytes < (1024*1024*1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
  else                              fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
  return fsize;
}
//changes JH 19/11/2022, added for formatting timestamp file
String Print_time(time_t timestamp) {
  char message[120];
  char buff[30];
  strftime(buff, 30, "%Y-%m-%d  %H:%M:%S", localtime(&timestamp));
  return buff;
}


//Download a file from the SD, it is called in void SD_dir()
void SD_file_download(String filename)
{
  if (sdOK) 
  { 
    File download = SD.open("/"+filename);
    if (download) 
    {
      downloading_file=true;
      filename.remove(0,1);//remove 1 character starting at index 0, this is /, after downloading -> _filename....
      server.sendHeader("Content-Disposition", "attachment; filename="+filename);
      server.sendHeader("Connection", "close");
      server.streamFile(download, "application/octet-stream");
      download.close();
    } else {ReportFileNotPresent("download");downloading_file=false;}
  } else{ ReportSDNotPresent();}
}

//Prints the directory, it is called in void SD_dir() 
void printDirectory(const char * dirname, uint8_t levels)
{
  File root = SD.open(dirname);

  if(!root){
    return;
  }
  if(!root.isDirectory()){
    return;
  }
  File file = root.openNextFile();
  int i = 0;
  while(file){
    time_t now;
    time(&now);
    if (webpage.length() > 1000) {
      SendHTML_Content();
    }
    if(file.isDirectory()){
      if(levels==1){
        if(String(file.name())== "/Archive"){
        webpage += "<tr>\n<td>"+String(file.isDirectory()?"Dir":"File")+"</td><td></td><td></td><td></td><td></td></tr>";
        printDirectory(file.name(), levels);//was levels-1
        }
      }
    }
    else if((config.archive_days>0)&(levels==2)&(file.getLastWrite()<(now-config.archive_days*24*3600))&(file.getLastWrite()>EPOCH_2022)){//older then xx days but younger then 2021, copy to archive, but not config.txt !!
        char buffer[40]="/Archive";//&(file.getLastWrite()>EPOCH_2022)
        strcat(buffer,file.name());//only rename if not Archive listing !!!
        if((String(file.name())!="config.txt")&(String(file.name())!= "/config.txt")&(String(file.name())!="/config_backup.txt")&(String(file.name())!="config_backup.txt")){
          SD.rename(file.name(),buffer); 
          } 
    }
    else
    {
      String filename =String(file.name());
      filename.remove(0,1);//remove 1 character starting at index 0
      webpage += "<tr><td width='20%'>"+filename+"</td>";
      int bytes = file.size();
      String fsize = "";
      time_t file_date = file.getLastWrite();//changes JH 19/11/2022
      String fd = Print_time(file_date);//Winscp MLSD expect UTC time !!!
      //if (bytes < 1024)                     fsize = String(bytes)+" B";
      //else if(bytes < (1024 * 1024))        fsize = String(bytes/1024.0,3)+" KB";
      //else if(bytes < (1024 * 1024 * 1024)) fsize = String(bytes/1024.0/1024.0,3)+" MB";
      //else                                  fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
      fsize = String(bytes/1024.0/1024.0,3)+" MB";//fsize always in MB !!!      
      webpage += "<td>"+fsize+"</td>";
      webpage += "<td>"+fd+"</td>"; //
      webpage += "<td>";
      webpage += F("<form action='/' method='post'>");
      webpage += F("<button type='submit' class='button' name='download'"); 
      webpage += F("' value='"); webpage +="download_"+String(file.name()); webpage +=F("'>Download</button>");
      webpage += F("</form>");
      webpage += "</td>";
      webpage += "<td>";
      if((String(file.name()) != "config.txt") & (String(file.name()) != "/config.txt") & (String(file.name()) != "/config_backup.txt") & (String(file.name()) != "config_backup.txt")){
        webpage += F("<form action='/' method='post'>"); 
        webpage += F("<button type='submit' name='delete' class='button_del' onclick='return confirmdelete();'"); 
        webpage += F("' value='"); webpage +="delete_"+String(file.name()); webpage += F("'>Delete</button>");
        webpage += F("</form>");
      }
      webpage += "</td>\n";
      webpage += "</tr>";

    }
    file = root.openNextFile();
    i++;
  }
  file.close();
}

//Delete a file from the SD, it is called in void SD_dir()
void SD_file_delete(String filename) 
{ 
  if (sdOK) { 
    //SendHTML_Header();
    File dataFile = SD.open("/"+filename, FILE_READ); //Now read data from SD Card 
    if (dataFile)
    {
      if (SD.remove("/"+filename)) {
        Serial.println(F("File deleted successfully"));
      }//toegevoegd
    }//toegevoegd
     }//toegevoegd
        /*
        webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
        webpage += "<tr><th>File '"+filename+"' has been erased</th></tr></table></div>"; 
        webpage += F("<a href='/'>[Back]</a><br><br>");
      }
      else
      { 
        webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
        webpage += F("<tr><th>File was not deleted - error</th></tr></table></div>");
        webpage += F("<a href='/'>[Back]</a><br><br>");
      }
    } else ReportFileNotPresent("delete");
    append_page_footer(); 
    SendHTML_Content();
    SendHTML_Stop();
  } else ReportSDNotPresent();
*/  
} 

/*********  FUNCTIONS  **********/
void SD_dir(int archive);
void SD_directory(void){
  SD_dir(0);
}
void SD_archive_list(void){
  SD_dir(1);
}
void SD_archive_file(void){
  SD_dir(2);
}
//Initial page of the server web, list directory and give you the chance of deleting and uploading
void SD_dir(int archive)
{
  if (sdOK) 
  {
    //Action acording to post, dowload or delete, by MC 2022
    if (server.args() > 0 ) //Arguments were received, ignored if there are not arguments
    { 
      Serial.println(server.arg(0));
  
      String Order = server.arg(0);
      Serial.println(Order);
      
      if (Order.indexOf("download_")>=0)
      {
        Order.remove(0,9);
        SD_file_download(Order);
        Serial.println(Order);
      }
  
      if ((server.arg(0)).indexOf("delete_")>=0)
      {
        Order.remove(0,7);
        SD_file_delete(Order);
        Serial.println(Order);
      }
    }

    File root = SD.open("/");
    if (root) {
      root.rewindDirectory();
      SendHTML_Header();   
      webpage += F("<table id='esplogger'>");
      webpage += F("<tr><th>Name</th><th>Size</th><th>Timestamp</th><th>Download</th><th>Delete</th></tr>");
      printDirectory("/",archive);
      webpage += F("\n</table>");
      SendHTML_Content();
      root.close();
    }
    else 
    {
      SendHTML_Header();
      webpage += F("<h3>No Files Found</h3>");
    }
    append_page_footer();
    SendHTML_Content();
    SendHTML_Stop();   //Stop is needed because no content length was sent
  } else ReportSDNotPresent();
}

//ReportCouldNotCreateFile
void ReportCouldNotCreateFile(String target)
{
  SendHTML_Header();
  webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
  webpage += F("<tr><th>Could Not Create Uploaded File (write-protected?)</th></tr></table></div>"); 
  webpage += F("<a href='/"); webpage += target + "'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
//Upload a file to the SD
void File_Upload()
{
  append_page_header();
  webpage += F("<FORM action='/fupload' method='post' enctype='multipart/form-data'>");
  webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
  webpage += F("<tr><th>select File to Upload</th><th>Action</th></tr>");
  webpage += F("<tr><td><input type='file' name='fupload' id = 'fupload' value=''></td><td>");
  webpage += F("<button class='button' type='submit'>Upload File</button></td></tr>");
  webpage += F("</table></div></FORM>");
  webpage += F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200, "text/html",webpage);
}

//Handles the file upload a file to the SD
File UploadFile;
//Upload a new file to the Filing system
void handleFileUpload()
{ 
  HTTPUpload& uploadfile = server.upload();
  if(uploadfile.status == UPLOAD_FILE_START)
  {
    String filename = uploadfile.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    Serial.print("Upload File Name: "); Serial.println(filename);
    SD.remove(filename);                         //Remove a previous version, otherwise data is appended the file again
    UploadFile = SD.open(filename, FILE_WRITE);  //Open the file for writing in SD (create it, if doesn't exist)
    filename = String();
  }
  else if (uploadfile.status == UPLOAD_FILE_WRITE)
  {
    if(UploadFile) UploadFile.write(uploadfile.buf, uploadfile.currentSize); // Write the received bytes to the file
  } 
  else if (uploadfile.status == UPLOAD_FILE_END)
  {
    if(UploadFile)          //If the file was successfully created
    {                                    
      UploadFile.close();   //Close the file again
      Serial.print("Upload Size: "); Serial.println(uploadfile.totalSize);
      webpage = "";
      append_page_header();
      webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
      webpage += F("<tr><th>File was successfully uploaded</th></tr>"); 
      webpage += F("<tr><th>Uploaded File Name: "); webpage += uploadfile.filename+"</th></tr>";
      webpage += F("<tr><th>File Size: "); webpage += file_size(uploadfile.totalSize) + "</th></tr></table></div>"; 
      webpage += F("<a href='/'>[Back]</a><br><br>");
      append_page_footer();
      server.send(200,"text/html",webpage);
    } 
    else
    {
      ReportCouldNotCreateFile("upload");
    }
  }
}

//Config Part - allow to change the config on txt by webinterface
void Config_TXT()
{
  append_page_header();
  webpage += html_config_header;
  html_config(webpage);
  webpage += html_config_footer;
  append_page_footer();
  server.send(200, "text/html",webpage);
}

void handleConfigUpload() {
 if (sdOK) 
  {
   SD.remove("/config_backup.txt");
   SD.rename("/config.txt", "/config_backup.txt");
   SD.remove("/config.txt");
  
    // Open file for writing
    File file = SD.open("/config.txt", FILE_WRITE);
    if (!file) {
      Serial.println(F("Failed to create file"));
      return;
    }
    StaticJsonDocument<1024> doc;
    // Set the values in the document
    Serial.println("calspeed:"+server.arg("cal_speed"));
    doc["cal_bat"] = serialized(server.arg("cal_bat")); 
    doc["cal_speed"] = serialized(server.arg("cal_speed")); 
    doc["sample_rate"] = server.arg("sample_rate").toInt();
    doc["gnss"] = server.arg("gnss").toInt();
    doc["archive_days"] = server.arg("archive_days").toInt(); 
    doc["logTXT"] = server.arg("logTXT").toInt(); 
    doc["logSBP"] = server.arg("logSBP").toInt(); 
    doc["logUBX"] = server.arg("logUBX").toInt();
    doc["logUBX_nav_sat"] = server.arg("logUBX_nav_sat").toInt();
    doc["logGPY"] = server.arg("logGPY").toInt();
    doc["logGPX"] = server.arg("logGPX").toInt();
    doc["file_date_time"] = server.arg("file_date_time").toInt();
    doc["dynamic_model"] = server.arg("dynamic_model").toInt();
    doc["GPIO12_screens"] = server.arg("GPIO12_screens").toInt();
    doc["timezone"] = serialized(server.arg("timezone")); 
    doc["UBXfile"] = server.arg("UBXfile");
    doc["Sleep_info"] = server.arg("Sleep_info");
    doc["ssid"] = server.arg("ssid");
    doc["password"] = server.arg("password");

    config.ublox_type=server.arg("GPS_Type").toInt();
    if(config.ublox_type==0xFF) {//not in config.txt but saved in EEPROM !!!)
      EEPROM.write(0, config.ublox_type);
      EEPROM.commit();
      }  
    // Pretty Serialize JSON to file
    if (serializeJsonPretty(doc, file) == 0) {
      Serial.println(F("Failed to write to file"));
      SendHTML_Header();
      webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
      webpage += F("<tr><th>failed to Upload the file</th></tr></table></div>"); 
      webpage += F("<a href='/config"); webpage += "'>[Back]</a><br><br>";
      append_page_footer();
      SendHTML_Content();
      SendHTML_Stop();  
    }else{
      // Close the file
      file.close();
      SendHTML_Header();
      webpage += F("<div style='overflow-x:auto;'><table id='esplogger'>");
      webpage += F("<tr><th>Upload was successful!</th></tr></table></div>"); 
      webpage += F("<a href='/config"); webpage += "'>[Back]</a><br><br>";
      append_page_footer();
      SendHTML_Content();
      SendHTML_Stop();  
      if(server.arg("reboot") == "yes"){
        Ublox_off();
        delay(1000);
        ESP.restart(); //restart as wanted
      }
    }
  }
}

//end SD Card webinterface download section

//begin OTA
/* Style */
String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:358px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}</style>";

/* Login page */
void makeLoginString(void){
  String  actual_SW=SW_version;
  String e_type=ESP_type;
  loginIndex = "";
  loginIndex += F("<form name=loginForm>");
  loginIndex += F("<h1>ESP32 Login</h1>");
  loginIndex += F("<h2>Actual firmware : "); loginIndex += actual_SW +"</h2>";
  loginIndex += F("<h2>"); loginIndex += e_type +"</h2>";
  loginIndex += F("<input name=userid placeholder='User ID'> ");
  loginIndex += F("<input name=pwd placeholder=Password type=Password> ");
  loginIndex += F("<input type=submit onclick=check(this.form) class=btn value=Login></form>");
  loginIndex += F("<script>");
  loginIndex += F("function check(form) {");
  loginIndex += F("if(form.userid.value=='admin' && form.pwd.value=='admin')");//hier wordt het paswoord bepaald !!
  loginIndex += F("{window.open('/serverIndex')}");
  loginIndex += F("else");
  loginIndex += F("{alert('Error Password or Username')}");
  loginIndex += F("}");
  loginIndex += F("</script>");loginIndex += style;
} 
/* Server Index Page */
//https://github.com/italocjs/ESP32_OTA_APMODE/blob/main/Main.cpp
String serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
"<label id='file-input' for='file'>   Choose file...</label>"
"<input type='submit' class=btn value='Update'>"
"<br><br>"
"<a href='/'>download files</a>"
"<div id='prg'></div>"
"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
"<script>"
"function sub(obj){"
"var fileName = obj.value.split('\\\\');"
"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
"};"
"$('form').submit(function(e){"
"e.preventDefault();"
"var form = $('#upload_form')[0];"
"var data = new FormData(form);"
"$.ajax({"
"url: '/update',"
"type: 'POST',"
"data: data,"
"contentType: false,"
"processData:false,"
"xhr: function() {"
"var xhr = new window.XMLHttpRequest();"
"xhr.upload.addEventListener('progress', function(evt) {"
"if (evt.lengthComputable) {"
"var per = evt.loaded / evt.total;"
"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
"$('#bar').css('width',Math.round(per*100) + '%');"
"}"
"}, false);"
"return xhr;"
"},"
"success:function(d, s) {"
"console.log('success!') "
"},"
"error: function (a, b, c) {"
"}"
"});"
"});"
"</script>" + style;

/* setup function */
void OTA_setup(void) {
  //Wifi connection already started !!
  /*use mdns for host name resolution*/

  /* add initialize sd card */
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error config up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/firmware", HTTP_GET, []() {
    makeLoginString();
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  //add webserver and fileupload part
  server.on("/", SD_directory);
  server.on("/archive_list", SD_archive_list);
  server.on("/archive_file", SD_archive_file);
  server.on("/upload",   File_Upload);
  server.on("/fupload",  HTTP_POST,[](){ server.send(200);}, handleFileUpload);
  server.on("/config",   Config_TXT);
  server.on("/configupload", handleConfigUpload);
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    Ublox_off();
    delay(1000);//to force the ublox again to default (9600 bd)
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
}
