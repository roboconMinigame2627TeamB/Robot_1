
#ifndef APPLICATION_H_
#define APPLICATION_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/

/*********************************************/
/*          Define                           */
/*********************************************/

/*********************************************/
/*          Enumarator                       */
/*********************************************/

/*********************************************/
/*          Variable                         */
/*********************************************/

typedef struct{
	unsigned char instruction;
	union{
		float data;
		struct{
			char byte1;
			char byte2;
			char byte3;
			char byte4;
		};
	}ins_buffer[4];
}ins_t;


typedef struct{
	unsigned char parameter;
	union{
		float data;
		struct{
			char byte1;
			char byte2;
			char byte3;
			char byte4;
		};
	}param_buffer[8];
}param_t;

typedef struct{
	unsigned char enquiry;
	union{
		float data;
		struct{
			char byte1;
			char byte2;
			char byte3;
			char byte4;
		};
	}enq_buffer[4];
}enq_t;

typedef struct{
	union {
		uint32_t LSA_ALL;
		struct{
			uint8_t lsa_F;
			uint8_t lsa_B;
			uint8_t lsa_L;
			uint8_t lsa_R;
		};
	};
}LSAdata_t;


typedef float ODN_data_t[8];

/*********************************************/
/*           Function Prototype              */
/*********************************************/

void APPApply(ins_t *pins);
void APPStart(ins_t *pins);
void APPResetPos(void);
void APPStop(void);
void PathPlanStop (void);
unsigned char APPBusy(ins_t *pins);
void APPSet(param_t *par);
void APPEnquire(enq_t *penq);
uint8_t APPPrintstatus(void);
void APPPrinting(char buff[]);
uint8_t APPFeedback(void);

#endif /* APPLICATION_H_ */
