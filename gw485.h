#include "esphome.h"

/*
v2 Test/Dev RS485 code both L01/L02 Pylon protocols. Do not use prior extensive testing
Pre-set to 24V 8S cell configuration
*/


char header2[] = { 0x7E, 0x32, 0x30, 0x30, 0x32, 0x34, 0x36, 0x30, 0x30 }; //reply
int timeoutcnt = 0;
uint8_t chargeflag = 0xC0;
uint8_t chargeimm = 0x18;
float lastwdt;
float currwdt;
int wdterr = 0;
int looprec = 0;
int activealm = 0;
uint8_t dflag = 0x10; // flag about data & alarms 0x11 data+alms , 0x10 data-alms
uint8_t cc = 0x00; // 00 normal  01 below 02 above
uint8_t pv = 0x00; // 00 normal  01 below 02 above
uint8_t dc = 0x00; // 00 normal  01 below 02 above

uint8_t raddr = 0x02; //0x02

int32_t ovp; //28000
int32_t uvp;
int32_t oct; //100 def
int32_t oct_dis; //100A def

int16_t chv = 3650;
int16_t clv = 3150;
int16_t cav = 3000;

int16_t cellhighestv = 0;
int16_t celllowestv = 0;
int16_t cellhighestn = 0;
int16_t celllowestn = 0;

uint16_t bvolt;

struct Cell {
    int16_t voltage;
} cellv[17];

struct Alarm {
    uint8_t status1;
    uint8_t status2;
    uint8_t status3;
    uint8_t status4;
    uint8_t status5;
} alarms;

struct Alarm62 {
    uint8_t status1;
    uint8_t status2;
    uint8_t status3;
    uint8_t status4;
} alarms62;

class gw485 : public PollingComponent, public UARTDevice {

public:
  gw485(UARTComponent *parent) : UARTDevice(parent) {}

char loutput[4];
char chksum[4];
char bout[6];

int clearBit(int n, int k) {
  return (n & (~(1 << (k-1))));  //k-1: index number of the bit to set
}

int setBit(int n, int k) {
  return (n | 1 << (k-1));  //k-1: index number of the bit to set
}

void to_6_bytes(char *out, uint32_t in) { //32
          sprintf(out, "%06X", in);
}

void to_4_bytes(char *out, uint16_t in) { //16
         sprintf(out, "%04X", in);
}

void to_2_bytes(char *out, uint8_t in) {
          sprintf(out, "%02X", in);
}

void to_1_byte(char *out, uint8_t in) {
          sprintf(out, "%01X", in);
}

void length_calc(int in, char *out) {

          int length = in;
          int len = length & 0xF;
          len = len + ((length >> 4) & 0xF);
          len = len + ((length >> 8) & 0xF);
          len = ((~len) & 0xF) + 1;
          len = len << 12;
          length = ((length + len) & 0xFFFF);
          sprintf((char*)(out), "%02X", length);
}

void checksum_calc(char *in, char *out, int sz) {

          int lbuf = sz;
          int cs1 = 0;
          for(int a=1; a<lbuf; a++) {
                  cs1 = cs1 + (int(in[a]));
          }
          cs1 = ~cs1;
          cs1 &= 0xFFFF;
          cs1 += 1;
          sprintf((char*)(out), "%02X", cs1);
}

void process_message(uint8_t *buffer, int l){ 
    
    if(id(bmscomms).state && buffer[0] == 0x7E && buffer[l-1] == 0x0D) {

        //zero all alarms
        alarms.status1 = 0x00;
        alarms.status2 = 0x00;
        alarms.status3 = 0x00;
        alarms.status4 = 0x00;
        alarms.status5 = 0x00;
        
        alarms62.status1 = 0x00;
        alarms62.status2 = 0x00;
        alarms62.status3 = 0x00;
        alarms62.status4 = 0x00;
                
        cc = 0x00; // 00 normal  01 below 02 above
        pv = 0x00; // 00 normal  01 below 02 above
        dc = 0x00; // 00 normal  01 below 02 above
        activealm = 0;
        
        //read cell V
        cellv[1].voltage = int(id(cv1).state*1000);
        cellv[2].voltage = int(id(cv2).state*1000);
        cellv[3].voltage = int(id(cv3).state*1000);
        cellv[4].voltage = int(id(cv4).state*1000);
        cellv[5].voltage = int(id(cv5).state*1000);
        cellv[6].voltage = int(id(cv6).state*1000);
        cellv[7].voltage = int(id(cv7).state*1000);
        cellv[8].voltage = int(id(cv8).state*1000);
        
        cellhighestv = int(id(maxcellv).state*1000);
        celllowestv = int(id(mincellv).state*1000);
    
        bvolt = int(id(bmstv).state*1000);
        ovp = int(id(prot_ov).state*1000);
        uvp = int(id(prot_uv).state*1000);
    
        if(bvolt <= 24200 || id(caprem).state < 20) {
            ESP_LOGD("GW485","BATT VERY LOW CHARGE IMMEDIATELY!");
            chargeflag = chargeflag ^ chargeimm;
            activealm = 1;
        } else if (bvolt > 25800 || id(caprem).state > 20 ) {
            chargeflag = 0xC0;
        }
        
        //Volt Alm
        //ESP_LOGD("GW485","V!!! vb:%d , vt:%d, vl:%d", bvolt, ovp, uvp);
        if(bvolt >= ovp) { //high
            alarms.status1 = setBit(alarms.status1, 1);
            alarms62.status1 = setBit(alarms62.status1, 6);
            alarms62.status3 = setBit(alarms62.status3, 8);
            pv = 0x02;
            activealm = 1;
        } else if (bvolt <= uvp){ //low
            alarms.status1 = setBit(alarms.status1, 8);
            alarms62.status1 = setBit(alarms62.status1, 6);
            alarms62.status3 = setBit(alarms62.status3, 7);
            pv = 0x01;
            activealm = 1;
        } else { //ok
            alarms.status1 = clearBit(alarms.status1,1);
            alarms.status1 = clearBit(alarms.status1,8);
            alarms62.status1 = clearBit(alarms62.status1, 6);
            alarms62.status3 = clearBit(alarms62.status3, 7);
            alarms62.status3 = clearBit(alarms62.status3, 8); 
        }
        //Curr Alm
        int32_t current = int(id(bmscurr).state*100); // actual value * 100 
        
        //current = 11000;
        
        oct = int(id(prot_ch_oc).state*100);
        int16_t oct_n = int(id(prot_ch_oc).state*-100);
        oct_dis = int(id(prot_dis_oc).state*100);
        if(current > 0 && (current >= oct)) { //chg oc
            alarms.status1 = setBit(alarms.status1, 3);
            alarms62.status2 = setBit(alarms62.status2, 7);
            cc = 0x02;
            activealm = 1;
        } else if (current < 0 && (current <= oct_n)){ //dis chg oc
            alarms.status1 = setBit(alarms.status1, 5);
            alarms62.status2 = setBit(alarms62.status2, 6);
            chargeflag = clearBit(chargeflag, 7);
            dc = 0x02;
            activealm = 1;
        } else {
            alarms.status1 = clearBit(alarms.status1,3);
            alarms.status1 = clearBit(alarms.status1,5);
            alarms62.status2 = clearBit(alarms62.status2, 6);
            alarms62.status2 = clearBit(alarms62.status2, 7);
            chargeflag = setBit(chargeflag, 7);
        }
        
        //ESP_LOGD("GW485","A!! c:%d , cc:%d, cd:%d", current , oct, oct_dis);
        //Tmp Alm
        int16_t tt = int(id(bmstmp).state);
        int16_t bt = int(id(prot_ch_thigh).state);
        int16_t bt_l = int(id(prot_dis_thigh).state)*-1;
        int16_t celltavg = (int(id(bmst1).state) + int(id(bmst2).state))/2;
        int16_t bt_cl = int(id(prot_ch_tlow).state);
    
        //celltavg = 61;
    
        if(tt >= bt) { //chg
            alarms.status1 = setBit(alarms.status1, 7);
            alarms62.status1 = setBit(alarms62.status1, 2);
            alarms62.status3 = setBit(alarms62.status3, 2);
            activealm = 1;
        } else if (tt <= bt_l) { //dis chg
            alarms.status1 = setBit(alarms.status1, 6);
            activealm = 1;
        } else {
            alarms.status1 = clearBit(alarms.status1,7);
            alarms.status1 = clearBit(alarms.status1,6);
            alarms62.status1 = clearBit(alarms62.status1, 2);
            alarms62.status3 = clearBit(alarms62.status3, 2);
        }
        
        if(celltavg >= bt) { //chg high temp
            alarms62.status1 = setBit(alarms62.status1, 4);
            alarms62.status3 = setBit(alarms62.status3, 4);
            activealm = 1;
        } else if (celltavg <= bt_cl) { //chg low temp
            alarms62.status1 = setBit(alarms62.status1, 3);
            alarms62.status3 = setBit(alarms62.status3, 3);
            activealm = 1;
        } else {
           alarms62.status1 = clearBit(alarms62.status1, 3);
           alarms62.status1 = clearBit(alarms62.status1, 4);
           alarms62.status3 = clearBit(alarms62.status3, 3);
           alarms62.status3 = clearBit(alarms62.status3, 4);
        }
    
        //SOC
        int16_t soc = (int(id(caprem).state)) + 10; //90% add up to 100%
        int32_t totalccc = int(id(totalcc).state);
        int32_t avgcc = totalccc;
        //ESP_LOGD("GW485","SOC:%d, +:%d", int(id(caprem).state), soc);


        if(activealm == 1) {
            //ESP_LOGD("GW485","THERE ARE ALARMS!!!!");
            //ESP_LOGD("GW485","ALARM BITS, 44 s1:%d, s4:%d, dflag: %d", alarms.status1,  alarms.status4, dflag);
            //ESP_LOGD("GW485","ALARM BITS, 62 s1:%d, s2:%d s3:%d s4:%d", alarms62.status1, alarms62.status2, alarms62.status3, alarms62.status4 );
            dflag = setBit(dflag,1);
            ESP_LOGD("GW485","A!! Bvolt:%d , curr:%d, dflag:%d, almb:%d, cc:%d, mv:%d, dc:%d", bvolt,current, dflag, alarms.status1, cc, pv, dc);

        } else {
            dflag = clearBit(dflag,1);
        }

        //Update error loop cntr
        currwdt = id(wdtrt).state;
        if(looprec == 0) {
            lastwdt = currwdt;
        }
        looprec++;
        if(looprec >= 240) {
         if(currwdt == lastwdt) {
            id(bmshtr).toggle();
         }
         looprec = 0;
         }
    
        std::string addr;
        addr += char(buffer[3]);
        addr += char(buffer[4]); // Addr
        
        std::string cid1;
        cid1 += char(buffer[5]);
        cid1 += char(buffer[6]); // CID1
            
        std::string cid2;
        cid2 += char(buffer[7]);
        cid2 += char(buffer[8]); // CID2

        if(addr != "02") {
                ESP_LOGD("GW485","Command not for us!");
        } else {
            
            if(cid1 == "46"){

                if(cid2 == "42") {

                    ESP_LOGD("GW485","42 - Battery Details");
                    std::string frame;
                    std::string rply;
            
                    char twobyte[4];
                    char onebyte[2];
                    char chksum1[4];
            
                    uint8_t cellcnt = 8;
                    uint8_t tempcnt = 3;
                      
                    uint32_t r_cap = (int(id(totalcaprem).state*1000) + 20000); //20Ah+ stop 90%
                    uint32_t t_cap = int(id(totalcap).state*1000);
            
                    frame.clear();
                    rply.clear();
                      
                    int alen = sizeof(header2) / sizeof(header2[0]);
                    for (int a=0; a<sizeof(header2); a++) {
                        frame += header2[a];
                    }
            
                    to_2_bytes(onebyte,dflag);
                    rply.append(onebyte);
                    
                    to_2_bytes(onebyte,raddr);
                    rply.append(onebyte);
                    
                    to_2_bytes(twobyte,cellcnt);
                    rply.append(twobyte);
            
                    for(int a=1; a<9; a++) {
                        to_4_bytes(twobyte, cellv[a].voltage);
                        rply.append(twobyte);
                    }

                    to_2_bytes(onebyte,tempcnt);
                    rply.append(onebyte);

                    to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731));
                    rply.append(twobyte);
                    
                    to_4_bytes(twobyte,(int(id(bmst1).state*10) + 2731));
                    rply.append(twobyte);
                    
                    to_4_bytes(twobyte,(int(id(bmst2).state*10) + 2731));
                    rply.append(twobyte);

                    //ESP_LOGD("GW485","Batt current: %d", current);
                    to_4_bytes(twobyte,current);
                    rply.append(twobyte);
            
                    to_4_bytes(twobyte,bvolt);
                    rply.append(twobyte);
            
                    uint16_t r_capacity = 0xFFFF;
                    to_4_bytes(twobyte,r_capacity);
                    rply.append(twobyte);
            
                    uint8_t bcnt65 = 4;
                    to_2_bytes(onebyte,bcnt65);
                    rply.append(onebyte);
            
                    uint16_t t_capacity = 0xFFFF;
                    to_4_bytes(twobyte,t_capacity);
                    rply.append(twobyte);
            
                    uint8_t ccnt = 2;
                    to_4_bytes(twobyte,ccnt);
                    rply.append(twobyte);
            
                    to_6_bytes(bout,r_cap);
                    rply.append(bout);
            
                    to_6_bytes(bout,t_cap);
                    rply.append(bout);
            
                    length_calc(rply.size(),loutput);
                    frame.append(loutput);
                    frame.append(rply);
            
                    char arr2[frame.size()];
                    alen = sizeof(arr2) / sizeof(arr2[0]);
                    for (int x = 0; x < alen; x++) {
                        arr2[x] = frame[x];
                    }
            
                    checksum_calc(arr2, chksum1, alen);
                    frame.append(chksum1);
            
                    frame += char(0x0D);
                    int flen = frame.size();
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                    this->parent_->write_array(ptr, flen);
                
                } else if(cid2 == "92") {
    
                    ESP_LOGD("GW485","92 - chg/dis info");
                      
                    std::string frame;
                    std::string rply;
            
                    char twobyte[4];
                    char onebyte[2];
                    char chksum1[4];
            
                    rply.clear();
                    frame.clear();
            
                    to_2_bytes(onebyte,raddr);
                    rply.append(onebyte);
            
                    to_4_bytes(twobyte,ovp);
                    rply.append(twobyte);
            
                    to_4_bytes(twobyte,uvp);
                    rply.append(twobyte);
            
                    to_4_bytes(twobyte,oct);
                    rply.append(twobyte);
            
                    to_4_bytes(twobyte,oct_dis);
                    rply.append(twobyte);
            
                    to_2_bytes(onebyte,chargeflag);
                    rply.append(onebyte);
            
                    int alen = sizeof(header2) / sizeof(header2[0]);
                    for (int a=0; a<sizeof(header2); a++) {
                           frame += header2[a];
                    }
            
                    length_calc(rply.size(),loutput);
                    frame.append(loutput);
                    frame.append(rply);

                    char arr3[frame.size()];
                    alen = sizeof(arr3) / sizeof(arr3[0]);
                    for (int x = 0; x < sizeof(arr3); x++) {
                        arr3[x] = frame[x];
                    }
                    checksum_calc(arr3, chksum1, sizeof(arr3));
                    frame.append(chksum1);
            
                    frame += char(0x0D);
                    int flen = frame.size();
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                    this->parent_->write_array(ptr, flen);

                } else if(cid2 == "93") { //93
                    
                    ESP_LOGD("GW485","Command - 93 - SN");
                    uint8_t sn[] = { 0x33, 0x30, 0x33, 0x31, 0x33, 0x32, 0x33, 0x33, 0x33, 0x34, 0x33, 0x35, 0x33, 0x36, 0x33, 0x37, 0x33, 0x38, 0x33, 0x39, 0x36, 0x31, 0x36, 0x32, 0x36, 0x33, 0x36, 0x34, 0x36, 0x35, 0x36, 0x36 };
                        
                    std::string frame;
                    std::string rply;
                    char twobyte[4];
                    char onebyte[2];
                    char chksum1[4];
                       
                    //uint8_t raddr = 0x02;
                                  
                    rply.clear();
                    frame.clear();
                        
                    to_2_bytes(onebyte,raddr);
                    rply.append(onebyte);
                        
                    int alen = sizeof(header2) / sizeof(header2[0]);
                    for (int a=0; a<alen; a++) {
                        frame += header2[a];
                    }
                        
                    alen = sizeof(sn) / sizeof(sn[0]);
                    for (int a=0; a<alen; a++) {
                        rply += sn[a];
                    }
                        
                    length_calc(rply.size(),loutput);
                    frame.append(loutput);
                    frame.append(rply);
                        
                    char arr4[frame.size()];
                    alen = sizeof(arr4) / sizeof(arr4[0]);
                    for (int x = 0; x <alen; x++) {
                             arr4[x] = frame[x];
                    }
                    checksum_calc(arr4, chksum1, alen);
                    frame.append(chksum1);
                    frame += char(0x0D);

                    printf("\nSending...%s\n", frame.c_str());
                        
                    int flen = frame.size();
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                    this->parent_->write_array(ptr, flen);
                    printf("Sent...\n");
                    
                } else if(cid2 == "4F") {
                        ESP_LOGD("GW485","Command - 4F - Protocol");
                    //    uint8_t header2[] = { 0x7E, 0x32, 0x30, 0x31, 0x32, 0x34, 0x36, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x46, 0x44, 0x42, 0x32, 0x0D }; //reply
                    //    write_array(header2, sizeof(header2));
                     //   printf("Sent...\n");
                    
                } else if(cid2 == "44") { 
                
                    ESP_LOGD("GW485","44 - Alarm info");
     
                    std::string frame;
                    std::string rply;
                    char twobyte[4];
                    char onebyte[2];
                    char chksum1[4];
                        
                    uint8_t cccnt = 8;
                    uint8_t cstat = 0x02; // 00 normal  01 below 02 above
                    uint8_t tmcnt = 2;
                       
                    rply.clear();
                    frame.clear();

                    to_2_bytes(onebyte,dflag);
                    rply.append(onebyte);
                    to_2_bytes(onebyte,raddr);
                    rply.append(onebyte);
                        
                    int alen = sizeof(header2) / sizeof(header2[0]);
                    for (int a=0; a<alen; a++) {
                        frame += header2[a];
                    }
                        
                    to_2_bytes(onebyte,cccnt); //no of cells
                    rply.append(onebyte);

                    for(int a=0; a< cccnt; a++) {
                        to_2_bytes(onebyte,cstat);
                        rply.append(onebyte);
                    }
                        
                    to_2_bytes(onebyte,tmcnt);
                    rply.append(onebyte);

                    for(int a=0; a<tmcnt; a++) {
                        to_2_bytes(onebyte,cstat);
                        rply.append(onebyte);
                    }
                                        
                    to_2_bytes(onebyte,cc);
                    rply.append(onebyte);
                    
                    to_2_bytes(onebyte,pv);
                    rply.append(onebyte);
                    
                    to_2_bytes(onebyte,dc);
                    rply.append(onebyte);
                    
                    to_2_bytes(onebyte,alarms.status1);
                    rply.append(onebyte);
                    to_2_bytes(onebyte,alarms.status2);
                    rply.append(onebyte);
                    to_2_bytes(onebyte,alarms.status3);
                    rply.append(onebyte);
                    to_2_bytes(onebyte,alarms.status4);
                    rply.append(onebyte);
                    to_2_bytes(onebyte,alarms.status5);
                    rply.append(onebyte);

                    length_calc(rply.size(),loutput);
                    frame.append(loutput);
                    frame.append(rply);

                    char arr4[frame.size()];
                    alen = sizeof(arr4) / sizeof(arr4[0]);
                    for (int x = 0; x <alen; x++) {
                        arr4[x] = frame[x];
                    }
                    
                    checksum_calc(arr4, chksum1, alen);
                    frame.append(chksum1);

                    frame += char(0x0D);
                    int flen = frame.size();
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                    this->parent_->write_array(ptr, flen);

                } else if (cid2 == "47") { //system parameters
                
                     ESP_LOGD("GW485","47 - System parameters");
                     
                     std::string frame;
                     std::string rply;
                     char twobyte[4];
                     char onebyte[2];
                     char chksum1[4];
                     
                     int32_t chth = int(id(prot_ch_thigh).state*100);
                     int32_t disth = int(id(prot_dis_thigh).state*100);
                     
                     int32_t chtl = int(id(prot_ch_tlow).state*100);
                     int32_t distl = int(id(prot_dis_tlow).state*100);
                     
                     oct = int(id(prot_ch_oc).state*100);
                     int32_t oct_n = int(id(prot_ch_oc).state*-100);
                     
                     rply.clear();
                     frame.clear();
                     
                    to_2_bytes(onebyte,dflag);
                    rply.append(onebyte);
                    to_2_bytes(onebyte,raddr);
                    rply.append(onebyte);
 
                     int alen = sizeof(header2) / sizeof(header2[0]);
                     for (int a=0; a<alen; a++) {
                         frame += header2[a];
                     }
                    
                    to_4_bytes(twobyte,chv);
                    rply.append(twobyte);
                    to_4_bytes(twobyte,clv);
                    rply.append(twobyte);
                    to_4_bytes(twobyte,cav);
                    rply.append(twobyte);
                     
                    to_4_bytes(twobyte, (chth + 2731));
                    rply.append(twobyte);
                    to_4_bytes(twobyte, (chtl + 2731));
                    rply.append(twobyte);                   
                    
                    //ESP_LOGD("GW485","BCURR %d", current);
                    to_4_bytes(twobyte,oct);
                    rply.append(twobyte);
                    
                    to_4_bytes(twobyte, ovp);
                    rply.append(twobyte);
                    to_4_bytes(twobyte, uvp+1000);
                    rply.append(twobyte);
                    to_4_bytes(twobyte, uvp);
                    rply.append(twobyte);
                    
                    //ESP_LOGD("GW485","th %d", (disth + 2731));
                    to_4_bytes(twobyte, (disth + 2731));
                    rply.append(twobyte);
                    
                    //ESP_LOGD("GW485","tl %d", (distl + 2731));
                    to_4_bytes(twobyte, (distl + 2731));
                    rply.append(twobyte);    
                    
                    to_4_bytes(twobyte,oct_dis);
                    rply.append(twobyte);

                    length_calc(rply.size(),loutput);
                    frame.append(loutput);
                    frame.append(rply);

                    char arr4[frame.size()];
                    alen = sizeof(arr4) / sizeof(arr4[0]);
                    for (int x = 0; x <alen; x++) {
                         arr4[x] = frame[x];
                    }
                        
                    checksum_calc(arr4, chksum1, alen);
                    frame.append(chksum1);
                    frame += char(0x0D);

                    int flen = frame.size();
                    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                    this->parent_->write_array(ptr, flen);

                } else if (cid2 == "60") {
                    
                    uint8_t bman[] = {  0x7E, 0x32, 0x30, 0x31, 0x32, 0x34, 0x36, 0x30, 0x30, 0x36, 0x30, 0x38, 0x32, 0x34, 0x36, 0x36, 0x46, 0x37, 0x32, 0x36, 0x33, 0x36, 0x35, 0x35, 0x46, 0x34, 0x43, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x35, 0x30,
                                        0x37, 0x39, 0x36, 0x43, 0x36, 0x46, 0x36, 0x45, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
                                        0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x39, 0x30, 0x32, 0x33, 0x30, 0x33, 0x31, 0x33, 0x32, 0x33, 0x33, 0x33, 0x34, 0x33, 0x35, 0x33, 0x36, 0x33, 0x37, 0x33, 0x38, 0x33, 0x39, 0x36, 0x31, 0x36, 0x32, 0x36,
                                        0x33, 0x36, 0x34, 0x36, 0x35, 0x36, 0x36, 0x33, 0x31, 0x33, 0x31, 0x33, 0x32, 0x33, 0x33, 0x33, 0x34, 0x33, 0x35, 0x33, 0x36, 0x33, 0x37, 0x33, 0x38, 0x33, 0x39, 0x36, 0x31, 0x36, 0x32, 0x36, 0x33, 0x36, 0x34,
                                        0x36, 0x35, 0x36, 0x36, 0x45, 0x33, 0x35, 0x33, 0x0D };
                    

                    write_array(bman, sizeof(bman));
                    printf("Sent...\n");
                    
                } else if(cid2 == "61") { // L04 battery analog info


                      ESP_LOGD("GW485","61 - Battery Details");
                      std::string frame;
                      std::string rply;
            
                      char twobyte[4];
                      char onebyte[2];
                      char chksum1[4];
            
                      uint8_t cellcnt = 8;
                      uint8_t tempcnt = 3;
                      
                      int16_t moduleaddr = 0x0102;
                      
                      frame.clear();
                      rply.clear();
                      
                      int alen = sizeof(header2) / sizeof(header2[0]);
                      for (int a=0; a<sizeof(header2); a++) {
                            frame += header2[a];
                      }
            
                      to_4_bytes(twobyte,bvolt);
                      rply.append(twobyte);
                    
                      to_4_bytes(twobyte,current);
                      rply.append(twobyte);
            
                      to_2_bytes(twobyte,soc);
                      rply.append(twobyte);
                      
                      to_4_bytes(twobyte,totalccc);
                      rply.append(twobyte);
                      
                      to_4_bytes(twobyte,totalccc);
                      rply.append(twobyte);
                                
                      to_2_bytes(twobyte,soc); //avg soc
                      rply.append(twobyte);
                      
                      to_2_bytes(twobyte,soc); //min soc
                      rply.append(twobyte);
                      
                      to_4_bytes(twobyte,cellhighestv);
                      rply.append(twobyte);
                      to_4_bytes(twobyte,moduleaddr);
                      rply.append(twobyte);
                      
                      to_4_bytes(twobyte,celllowestv);
                      rply.append(twobyte);
                      to_4_bytes(twobyte,moduleaddr);
                      rply.append(twobyte);
          
                       to_4_bytes(twobyte,(int(id(bmst1).state*10) + 2731)); //avg temp cell
                       rply.append(twobyte);
                       to_4_bytes(twobyte,(int(id(bmst1).state*10) + 2731)); //max temp cell
                       rply.append(twobyte);
                       to_4_bytes(twobyte,moduleaddr);
                       rply.append(twobyte);
            
                       to_4_bytes(twobyte,(int(id(bmst1).state*10) + 2731)); //minn temp cell
                       rply.append(twobyte);
                       to_4_bytes(twobyte,moduleaddr);
                       rply.append(twobyte);
            
                       to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731)); //bms avg
                       rply.append(twobyte);           
                       to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731)); //bms max
                       rply.append(twobyte);           
                       to_4_bytes(twobyte,moduleaddr);
                       rply.append(twobyte);
                       
                       to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731)); //bms min
                       rply.append(twobyte);           
                       to_4_bytes(twobyte,moduleaddr);
                       rply.append(twobyte);
                       
                       to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731)); //bms avg
                       rply.append(twobyte);           
                       to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731)); //bms max
                       rply.append(twobyte);           
                       to_4_bytes(twobyte,moduleaddr);
                       rply.append(twobyte);
                       
                       to_4_bytes(twobyte,(int(id(bmstmp).state*10) + 2731)); //bms min
                       rply.append(twobyte);           
                       to_4_bytes(twobyte,moduleaddr);
                       rply.append(twobyte);
           
                       length_calc(rply.size(),loutput);
                        frame.append(loutput);
                        frame.append(rply);

                        char arr4[frame.size()];
                        alen = sizeof(arr4) / sizeof(arr4[0]);
                        for (int x = 0; x <alen; x++) {
                                 arr4[x] = frame[x];
                        }
                        
                        checksum_calc(arr4, chksum1, alen);
                        frame.append(chksum1);
                        frame += char(0x0D);

                        int flen = frame.size();
                        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                        this->parent_->write_array(ptr, flen);
          
                    } else if(cid2 == "62") {
                    
                        ESP_LOGD("GW485","62 - Alarm data");

                        std::string frame;
                        std::string rply;
                        char twobyte[4];
                        char onebyte[2];
                        char chksum1[4];
                        
                        rply.clear();
                        frame.clear();

                        int alen = sizeof(header2) / sizeof(header2[0]);
                        for (int a=0; a<alen; a++) {
                            frame += header2[a];
                        }
                        
                        to_2_bytes(twobyte,alarms62.status1);
                        rply.append(twobyte);
                            
                        to_2_bytes(twobyte,alarms62.status2);
                        rply.append(twobyte);
    
                        to_2_bytes(twobyte,alarms62.status3);
                        rply.append(twobyte);
    
                        to_2_bytes(twobyte,alarms62.status4);
                        rply.append(twobyte);

                        length_calc(rply.size(),loutput);
                        frame.append(loutput);
                        frame.append(rply);

                        char arr4[frame.size()];
                        alen = sizeof(arr4) / sizeof(arr4[0]);
                        for (int x = 0; x <alen; x++) {
                                 arr4[x] = frame[x];
                        }
                        
                        checksum_calc(arr4, chksum1, alen);
                        frame.append(chksum1);

                        frame += char(0x0D);
                        int flen = frame.size();
                        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                        this->parent_->write_array(ptr, flen);                    

                    } else if(cid2 == "63") {
                    
                          ESP_LOGD("GW485","63 - chg/dis management");
                          std::string frame;
                          std::string rply;
                
                          char twobyte[4];
                          char onebyte[2];
                          char chksum1[4];
                
                          frame.clear();
                          rply.clear();
                          
                          to_4_bytes(twobyte,ovp);
                          rply.append(twobyte);
                    
                          to_4_bytes(twobyte,uvp);
                          rply.append(twobyte);
                    
                          to_4_bytes(twobyte,oct);
                          rply.append(twobyte);
                  
                          to_4_bytes(twobyte,oct_dis);
                          rply.append(twobyte);
    /*
                          for(int a=0;a<rply.size();a++) {
                                ESP_LOGD("GW485","OCT :%x", rply[a]);
                          }
     */              
      //                    ESP_LOGD("GW485","CF :%x", chargeflag);
                          to_2_bytes(onebyte,chargeflag);
                          rply.append(onebyte);
                                          
                          
                          int alen = sizeof(header2) / sizeof(header2[0]);
                          for (int a=0; a<sizeof(header2); a++) {
                                frame += header2[a];
                          }

                            length_calc(rply.size(),loutput);
                            frame.append(loutput);
                            frame.append(rply);
                            char arr4[frame.size()];
                            alen = sizeof(arr4) / sizeof(arr4[0]);
                            for (int x = 0; x <alen; x++) {
                                     arr4[x] = frame[x];
                            }
                            
                            checksum_calc(arr4, chksum1, alen);
                            frame.append(chksum1);
                            frame += char(0x0D);
                            int flen = frame.size();
                            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(frame.c_str());
                            this->parent_->write_array(ptr, flen);
                        
                    } else {
                          ESP_LOGD("GW485","CID2: %s", cid2.c_str());
                    }
                } //cid46
            } //adr02
    } else {
        ESP_LOGD("GW485","BMS Not Connected or packet malformed");
        ESP_LOGD("GW485","%01X, %01X, %01X", buffer[0], buffer[l], buffer[l-1]);
        timeoutcnt++;
        if(timeoutcnt > 30) {
            timeoutcnt = 0;
            id(bmshtr).toggle();
        }
    }
}

void setup() override {
    this->set_update_interval(1000);
    ESP_LOGD("GW485","Setup UART");
}


void loop() override {
    
    int availbleBytes = available();
    if (availbleBytes > 0) {
        uint8_t b[availbleBytes];
        read_array(b, availbleBytes);
        process_message(b, availbleBytes);
    }
}

void update() override {}

};