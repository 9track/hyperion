/* COMM3705.C   (C) Copyright Roger Bowler, 2008-2012                */
/*              (C) Copyright MHP, 2007 (see below)                  */
/*              (C) Copyright Matt Burke, 2020 (DLSw support)        */
/*              Hercules 3705 communications controller              */
/*              running NCP                                          */
/*                                                                   */
/*   Released under "The Q Public License Version 1"                 */
/*   (http://www.hercules-390.org/herclic.html) as modifications to  */
/*   Hercules.                                                       */

/***********************************************************************/
/*                                                                     */
/* comm3705.c - (C) Copyright 2007 by MHP <ikj1234i at yahoo dot com>  */
/*                                                                     */
/* Loosely based on commadpt.c by Ivan Warren                          */
/*                                                                     */
/* This module appears to the "host" as a 3705 communications          */
/* controller running NCP.  It does not attempt to provide an emulated */
/* execution environment for native 3705 code.                         */
/*                                                                     */
/* Experimental release 0.02 Oct. 15, 2007                             */
/*                                                                     */
/* A very minimalistic SNA engine is implemented.  All received SNA    */
/* requests are responded to with a positive response.  Also, there's  */
/* enough code to enable a single SNA session to logon in LU1 (TTY)    */
/* mode.                                                               */
/*                                                                     */
/* FID1 is the only SNA header type supported.                         */
/*                                                                     */
/* A large amount of required SNA functionality is not present in this */
/* release.  There are no "state machines", "layers", "services",      */
/* chaining*, pacing, brackets, etc.etc.etc.  There are                */
/* probably more bugs than working functions...    Enjoy ;-)           */
/*                                                                     */
/* A better implementation might be to separate the SNA functions out  */
/* into an independent process, with communications over a full-duplex */
/* TCP/IP socket... We might also get rid of all the magic constants...*/
/*                                                                     */
/* New in release 0.02 -                                               */
/* - VTAM switched (dial) support                                      */
/* - New remote NCP capability                                         */
/* - SNA 3270 (LU2) support (*with RU chaining)                        */
/* - fixes for some bugs in 0.01                                       */
/* New in release 0.03 -                                               */
/* - don't process TTY lines until CR received                         */
/* New in release 0.04 -                                               */
/* - make debug messages optional                                      */
/*                                                                     */
/*                      73 DE KA1RBI                                   */
/***********************************************************************/

#include "hstdinc.h"
#include "hercules.h"
#include "devtype.h"
#include "opcode.h"
#include "parser.h"
#include "comm3705.h"

#if defined(WIN32) && !defined(HDL_USE_LIBTOOL) && !defined(_MSVC_)
  SYSBLK *psysblk;
  #define sysblk (*psysblk)
#endif

#if !defined(min)
#define  min(a,b)              (((a) <= (b)) ? (a) : (b))
#endif

#if 1
#define DLSW_DEBUG(...)        logmsg(__VA_ARGS__)
#else
#define DLSW_DEBUG(...)        1 ? ((void)0) : logmsg
#endif

enum fid_remap {
    MAP_FID1_FID2,
    MAP_FID2_FID1
};

void make_seq (COMMADPT*, BYTE*);
static void make_sna_requests2(COMMADPT*);
static void sna_sig(COMMADPT*);
static void sna_reqcont(COMMADPT*, BYTE, U32);
static void sna_inop(COMMADPT*);
static void th_remap(enum fid_remap, BYTE*, U16);

static unsigned char R010201[3] = {0x01, 0x02, 0x01};
static unsigned char R010202[3] = {0x01, 0x02, 0x02};
static unsigned char R010203[3] = {0x01, 0x02, 0x03};
static unsigned char R010204[3] = {0x01, 0x02, 0x04};
static unsigned char R010205[3] = {0x01, 0x02, 0x05};
static unsigned char R01020A[3] = {0x01, 0x02, 0x0A};
static unsigned char R01020B[3] = {0x01, 0x02, 0x0B};
static unsigned char R01020F[3] = {0x01, 0x02, 0x0F};
static unsigned char R010211[3] = {0x01, 0x02, 0x11};
static unsigned char R010216[3] = {0x01, 0x02, 0x16};
static unsigned char R010217[3] = {0x01, 0x02, 0x17};
static unsigned char R010219[3] = {0x01, 0x02, 0x19};
static unsigned char R01021A[3] = {0x01, 0x02, 0x1A};
static unsigned char R01021B[3] = {0x01, 0x02, 0x1B};
static unsigned char R010280[3] = {0x01, 0x02, 0x80};
static unsigned char R010281[3] = {0x01, 0x02, 0x81};
static unsigned char R010284[3] = {0x01, 0x02, 0x84};

static unsigned char R010206[3] = {0x01, 0x02, 0x06};
static unsigned char R010207[3] = {0x01, 0x02, 0x07};
static unsigned char R010208[3] = {0x01, 0x02, 0x08};
static unsigned char R010209[3] = {0x01, 0x02, 0x09};
static unsigned char R01020C[3] = {0x01, 0x02, 0x0C};
static unsigned char R01020D[3] = {0x01, 0x02, 0x0D};
static unsigned char R01020E[3] = {0x01, 0x02, 0x0E};
static unsigned char R010214[3] = {0x01, 0x02, 0x14};
static unsigned char R010215[3] = {0x01, 0x02, 0x15};
static unsigned char R010218[3] = {0x01, 0x02, 0x18};
static unsigned char R010301[3] = {0x01, 0x03, 0x01};
static unsigned char R010302[3] = {0x01, 0x03, 0x02};
static unsigned char R010303[3] = {0x01, 0x03, 0x03};
static unsigned char R010331[3] = {0x01, 0x03, 0x31};
static unsigned char R010334[3] = {0x01, 0x03, 0x34};
static unsigned char R010380[3] = {0x01, 0x03, 0x80};
static unsigned char R010381[3] = {0x01, 0x03, 0x81};
static unsigned char R010382[3] = {0x01, 0x03, 0x82};
static unsigned char R010383[3] = {0x01, 0x03, 0x83};
static unsigned char R010480[3] = {0x01, 0x04, 0x80};
static unsigned char R010604[3] = {0x01, 0x06, 0x04};
static unsigned char R010681[3] = {0x01, 0x06, 0x81};
static unsigned char R410210[3] = {0x41, 0x02, 0x10};
static unsigned char R410222[3] = {0x41, 0x02, 0x22};
static unsigned char R410285[3] = {0x41, 0x02, 0x85};
static unsigned char R410304[3] = {0x41, 0x03, 0x04};
static unsigned char R410384[3] = {0x41, 0x03, 0x84};
static unsigned char R810601[3] = {0x81, 0x06, 0x01};
static unsigned char R810602[3] = {0x81, 0x06, 0x02};
static unsigned char R810620[3] = {0x81, 0x06, 0x20};
static unsigned char R810629[3] = {0x81, 0x06, 0x29};
static unsigned char R810680[3] = {0x81, 0x06, 0x80};
static unsigned char R810681[3] = {0x81, 0x06, 0x81};
static unsigned char R810685[3] = {0x81, 0x06, 0x85};
static unsigned char R818620[3] = {0x81, 0x86, 0x20};
static unsigned char R818627[3] = {0x81, 0x86, 0x27};
static unsigned char R818640[3] = {0x81, 0x86, 0x40};
static unsigned char R818641[3] = {0x81, 0x86, 0x41};
static unsigned char R818643[3] = {0x81, 0x86, 0x43};
static unsigned char R818645[3] = {0x81, 0x86, 0x45};
static unsigned char R818646[3] = {0x81, 0x86, 0x46};
static unsigned char R818647[3] = {0x81, 0x86, 0x47};
static unsigned char R818648[3] = {0x81, 0x86, 0x48};
static unsigned char R818649[3] = {0x81, 0x86, 0x49};
static unsigned char R81864A[3] = {0x81, 0x86, 0x4A};
static unsigned char R81864B[3] = {0x81, 0x86, 0x4B};

#define BUFPD 0x1C

static BYTE commadpt_immed_command[256]=
{ 0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*---------------------------------------------------------------*/
/* PARSER TABLES                                                 */
/*---------------------------------------------------------------*/
static PARSER ptab[]={
    {"lport",   PARSER_STR_TYPE},
    {"lhost",   PARSER_STR_TYPE},
    {"rport",   PARSER_STR_TYPE},
    {"rhost",   PARSER_STR_TYPE},
    {"dial",    PARSER_STR_TYPE},
    {"rto",     PARSER_STR_TYPE},
    {"pto",     PARSER_STR_TYPE},
    {"eto",     PARSER_STR_TYPE},
    {"switched",PARSER_STR_TYPE},
    {"lnctl",   PARSER_STR_TYPE},
    {"debug",   PARSER_STR_TYPE},
    {"emu3791", PARSER_STR_TYPE},
    {"locsuba", PARSER_STR_TYPE},
    {"rmtsuba", PARSER_STR_TYPE},
    {"locncpnm",PARSER_STR_TYPE},
    {"rmtncpnm",PARSER_STR_TYPE},
    {"idblk",   PARSER_STR_TYPE},
    {"idnum",   PARSER_STR_TYPE},
    {"unitsz",  PARSER_STR_TYPE},
    {"ackspeed",PARSER_STR_TYPE},
    {"dlsw",    PARSER_STR_TYPE},
    {NULL,NULL}
};

enum {
    COMMADPT_KW_LPORT=1,
    COMMADPT_KW_LHOST,
    COMMADPT_KW_RPORT,
    COMMADPT_KW_RHOST,
    COMMADPT_KW_DIAL,
    COMMADPT_KW_READTO,
    COMMADPT_KW_POLLTO,
    COMMADPT_KW_ENABLETO,
    COMMADPT_KW_SWITCHED,
    COMMADPT_KW_LNCTL,
    COMMADPT_KW_DEBUG,
    COMMADPT_KW_EMU3791,
    COMMADPT_KW_LOCSUBA,
    COMMADPT_KW_RMTSUBA,
    COMMADPT_KW_LOCNCPNM,
    COMMADPT_KW_RMTNCPNM,
    COMMADPT_KW_IDBLK,
    COMMADPT_KW_IDNUM,
    COMMADPT_KW_UNITSZ,
    COMMADPT_KW_ACKSPEED,
    COMMADPT_KW_DLSW
} comm3705_kw;

//////////////////////////////////////////////////////////////////////
// some code copied from console.c
static  HOST_INFO  cons_hostinfo;       /* Host info for this system */
/*-------------------------------------------------------------------*/
/* Telnet command definitions                                        */
/*-------------------------------------------------------------------*/
#define BINARY          0       /* Binary Transmission */
#define IS              0       /* Used by terminal-type negotiation */
#define SEND            1       /* Used by terminal-type negotiation */
#define ECHO_OPTION     1       /* Echo option */
#define SUPPRESS_GA     3       /* Suppress go-ahead option */
#define TIMING_MARK     6       /* Timing mark option */
#define TERMINAL_TYPE   24      /* Terminal type option */
#define NAWS            31      /* Negotiate About Window Size */
#define EOR             25      /* End of record option */
#define EOR_MARK        239     /* End of record marker */
#define SE              240     /* End of subnegotiation parameters */
#define NOP             241     /* No operation */
#define DATA_MARK       242     /* The data stream portion of a Synch.
                                   This should always be accompanied
                                   by a TCP Urgent notification */
#define BRK             243     /* Break character */
#define IP              244     /* Interrupt Process */
#define AO              245     /* Abort Output */
#define AYT             246     /* Are You There */
#define EC              247     /* Erase character */
#define EL              248     /* Erase Line */
#define GA              249     /* Go ahead */
#define SB              250     /* Subnegotiation of indicated option */
#define WILL            251     /* Indicates the desire to begin
                                   performing, or confirmation that
                                   you are now performing, the
                                   indicated option */
#define WONT            252     /* Indicates the refusal to perform,
                                   or continue performing, the
                                   indicated option */
#define DO              253     /* Indicates the request that the
                                   other party perform, or
                                   confirmation that you are expecting
                                   the other party to perform, the
                                   indicated option */
#define DONT            254     /* Indicates the demand that the
                                   other party stop performing,
                                   or confirmation that you are no
                                   longer expecting the other party
                                   to perform, the indicated option */
#define IAC             255     /* Interpret as Command */

/*-------------------------------------------------------------------*/
/* 3270 definitions                                                  */
/*-------------------------------------------------------------------*/

/* 3270 remote commands */
#define R3270_EAU       0x6F            /* Erase All Unprotected     */
#define R3270_EW        0xF5            /* Erase/Write               */
#define R3270_EWA       0x7E            /* Erase/Write Alternate     */
#define R3270_RB        0xF2            /* Read Buffer               */
#define R3270_RM        0xF6            /* Read Modified             */
#define R3270_RMA       0x6E            /* Read Modified All         */
#define R3270_WRT       0xF1            /* Write                     */
#define R3270_WSF       0xF3            /* Write Structured Field    */

/* 3270 orders */
#define O3270_SBA       0x11            /* Set Buffer Address        */
#define O3270_SF        0x1D            /* Start Field               */
#define O3270_SFE       0x29            /* Start Field Extended      */
#define O3270_SA        0x28            /* Set Attribute             */
#define O3270_IC        0x13            /* Insert Cursor             */
#define O3270_MF        0x2C            /* Modify Field              */
#define O3270_PT        0x05            /* Program Tab               */
#define O3270_RA        0x3C            /* Repeat to Address         */
#define O3270_EUA       0x12            /* Erase Unprotected to Addr */
#define O3270_GE        0x08            /* Graphic Escape            */

/* Inbound structured fields */
#define SF3270_AID      0x88            /* Aid value of inbound SF   */
#define SF3270_3270DS   0x80            /* SFID of 3270 datastream SF*/

/*-------------------------------------------------------------------*/
/* DLSw definitions                                                  */
/*-------------------------------------------------------------------*/

/* Message types */

#define CANUREACH       0x03                            /* Can U Reach Station */
#define ICANREACH       0x04                            /* I Can Reach Station */
#define REACH_ACK       0x05                            /* Reach Acknowledgment */
#define DGRMFRAME       0x06                            /* Datagram Frame */
#define XIDFRAME        0x07                            /* XID Frame */
#define CONTACT         0x08                            /* Contact Remote Station */
#define CONTACTED       0x09                            /* Remote Station Contacted */
#define RESTART_DL      0x10                            /* Restart Data Link */
#define DL_RESTARTED    0x11                            /* Data Link Restarted */
#define ENTER_BUSY      0x0C                            /* Enter Busy */
#define EXIT_BUSY       0x0D                            /* Exit Busy */
#define INFOFRAME       0x0A                            /* Information (I) Frame */
#define HALT_DL         0x0E                            /* Halt Data Link */
#define DL_HALTED       0x0F                            /* Data Link Halted */
#define NETBIOS_NQ      0x12                            /* NETBIOS Name Query */
#define NETBIOS_NR      0x13                            /* NETBIOS Name Recog */
#define DATAFRAME       0x14                            /* Data Frame */
#define HALT_DL_NOACK   0x19                            /* Halt Data Link with no Ack */
#define NETBIOS_ANQ     0x1A                            /* NETBIOS Add Name Query */
#define NETBIOS_ANR     0x1B                            /* NETBIOS Add Name Response */
#define KEEPALIVE       0x1D                            /* Transport Keepalive Message */
#define CAP_EXCHANGE    0x20                            /* Capabilities Exchange */
#define IFCM            0x21                            /* Independent Flow Control Message */
#define TEST_CIRC_REQ   0x7A                            /* Test Circuit Request */
#define TEST_CIRC_RSP   0x7B                            /* Test Circuit Response */

/* SSP flags */

#define SSPex           0x80                            /* explorer message */

/* Frame direction */

#define DIR_TGT         0x01                            /* origin to target */
#define DIR_ORG         0x02                            /* target to origin */

/* Header constants */

#define DLSW_VER        0x31                            /* DLSw version 1 */
#define LEN_CTRL        72                              /* control header length */
#define LEN_INFO        16                              /* info header length */

#define DLSW_PORT       2065

/* Common header fields */

#define HDR_VER         0x00                            /* Version Number */
#define HDR_HLEN        0x01                            /* Header Length */
#define HDR_MLEN        0x02                            /* Message Length */
#define HDR_RDLC        0x04                            /* Remote Data Link Correlator */
#define HDR_RDPID       0x08                            /* Remote DLC Port ID */
#define HDR_MTYP        0x0E                            /* Message Type */
#define HDR_FCB         0x0F                            /* Flow Control Byte */

/* Control header fields */

#define HDR_PID         0x10                            /* Protocol ID */
#define HDR_NUM         0x11                            /* Header Number */
#define HDR_LFS         0x14                            /* Largest Frame Size */
#define HDR_SFLG        0x15                            /* SSP Flags */
#define HDR_CP          0x16                            /* Circuit Priority */
#define HDR_TMAC        0x18                            /* Target MAC Address */
#define HDR_OMAC        0x1E                            /* Origin MAC Address */
#define HDR_OSAP        0x24                            /* Origin Link SAP */
#define HDR_TSAP        0x25                            /* Target Link SAP */
#define HDR_DIR         0x26                            /* Frame Direction */
#define HDR_DLEN        0x2A                            /* DLC Header Length */
#define HDR_ODPID       0x2C                            /* Origin DLC Port ID */
#define HDR_ODLC        0x30                            /* Origin Data Link Correlator */
#define HDR_OTID        0x34                            /* Origin Transport ID */
#define HDR_TDPID       0x38                            /* Target DLC Port ID */
#define HDR_TDLC        0x3C                            /* Target Data Link Correlator */
#define HDR_TTID        0x40                            /* Target Transport ID */

/* Flow control fields */

#define FCB_FCI         0x80                            /* Flow control indicator */
#define FCB_FCA         0x40                            /* Flow control acknowledge */
#define FCB_FCO         0x07                            /* Flow control operator */

#define FCO_RPT         0x00                            /* Repeat window operator */
#define FCO_INC         0x01                            /* Increment window operator */
#define FCO_DEC         0x02                            /* Decrement window operator */
#define FCO_RST         0x03                            /* Reset window operator */
#define FCO_HLV         0x04                            /* Halve window operator */

/* Capabilities Exchange Subfields */

#define CAP_VID         0x81                            /* Vendor ID */
#define CAP_VER         0x82                            /* DLSw Version */
#define CAP_IPW         0x83                            /* Initial Pacing Window */
#define CAP_VERS        0x84                            /* Version String */
#define CAP_MACX        0x85                            /* MAC Address Exclusivity */
#define CAP_SSL         0x86                            /* Supported SAP List */
#define CAP_TCP         0x87                            /* TCP Connections */
#define CAP_NBX         0x88                            /* NetBIOS Name Exclusivity */
#define CAP_MACL        0x89                            /* MAC Address List */
#define CAP_NBL         0x8A                            /* NetBIOS Name List */
#define CAP_VC          0x8B                            /* Vendor Context */

/*-------------------------------------------------------------------*/
/* Internal macro definitions                                        */
/*-------------------------------------------------------------------*/

/* DEBUG_LVL: 0 = none
              1 = status
              2 = headers
              3 = buffers
*/
#define DEBUG_LVL        0

#if DEBUG_LVL == 0
  #define TNSDEBUG1      1 ? ((void)0) : logmsg
  #define TNSDEBUG2      1 ? ((void)0) : logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 1
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      1 ? ((void)0) : logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 2
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      logmsg
  #define TNSDEBUG3      1 ? ((void)0) : logmsg
#endif
#if DEBUG_LVL == 3
  #define TNSDEBUG1      logmsg
  #define TNSDEBUG2      logmsg
  #define TNSDEBUG3      logmsg
#endif

#define BUFLEN_3270     65536           /* 3270 Send/Receive buffer  */
#define BUFLEN_1052     150             /* 1052 Send/Receive buffer  */


#undef  FIX_QWS_BUG_FOR_MCS_CONSOLES

/* Packet data access macros */

#define GET16(p,w)      (((U16) p[w]) | \
                        (((U16) p[(w)+1]) << 8))
#define GET32(p,w)      (((U32) p[w]) | \
                        (((U32) p[(w)+1]) << 8) | \
                        (((U32) p[(w)+2]) << 16) | \
                        (((U32) p[(w)+3]) << 24))

#define PUT16(p,w,x)    p[w] = (x) & 0xFF; \
                        p[(w)+1] = ((x) >> 8) & 0xFF
#define PUT32(p,w,x)    p[w] = (x) & 0xFF; \
                        p[(w)+1] = ((x) >> 8) & 0xFF; \
                        p[(w)+2] = ((x) >> 16) & 0xFF; \
                        p[(w)+3] = ((x) >> 24) & 0xFF

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO TRACE THE CONTENTS OF AN ASCII MESSAGE PACKET       */
/*-------------------------------------------------------------------*/
#if DEBUG_LVL == 3
static void
packet_trace( BYTE* pAddr, int iLen )
{
    int           offset;
    unsigned int  i;
    u_char        c = '\0';
    u_char        e = '\0';
    u_char        print_ascii[17];
    u_char        print_ebcdic[17];
    u_char        print_line[64];
    u_char        tmp[32];

    for( offset = 0; offset < iLen; )
    {
        memset( print_ascii,  0, sizeof( print_ascii  ) );
        memset( print_ebcdic, 0, sizeof( print_ebcdic ) );
        memset( print_line,   0, sizeof( print_line   ) );

        sprintf(print_line, "+%4.4X  ", offset );

        for( i = 0; i < 16; i++ )
        {
            c = *pAddr++;

            if( offset < iLen )
            {
                sprintf( tmp, "%2.2X", c );
                STRLCAT( print_line, tmp );

                print_ebcdic[i] = print_ascii[i] = '.';
                e = guest_to_host( c );

                if( isprint( e ) )
                    print_ebcdic[i] = e;
                if( isprint( c ) )
                    print_ascii[i] = c;
            }
            else
            {
                STRLCAT( print_line, "  " );
            }

            offset++;
            if( ( offset & 3 ) == 0 )
            {
                STRLCAT( print_line, " " );
            }
        }

        logmsg( "%s %s %s\n", print_line, print_ascii, print_ebcdic );
    }
} /* end function packet_trace */
#else
 #define packet_trace( _addr, _len)
#endif


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO DOUBLE UP ANY IAC BYTES IN THE DATA STREAM          */
/* Returns the new length after inserting extra IAC bytes            */
/*-------------------------------------------------------------------*/
static int
double_up_iac (BYTE *buf, int len)
{
int     m, n, x, newlen;

    /* Count the number of IAC bytes in the data */
    for (x=0, n=0; n < len; n++)
        if (buf[n] == IAC) x++;

    /* Exit if nothing to do */
    if (x == 0) return len;

    /* Insert extra IAC bytes backwards from the end of the buffer */
    newlen = len + x;
    TNSDEBUG3("console: DBG002: %d IAC bytes added, newlen=%d\n",
            x, newlen);
    for (n=newlen, m=len; n > m; ) {
        buf[--n] = buf[--m];
        if (buf[n] == IAC) buf[--n] = IAC;
    }
    packet_trace (buf, newlen);
    return newlen;

} /* end function double_up_iac */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO SEND A DATA PACKET TO THE CLIENT                    */
/*-------------------------------------------------------------------*/
static int
send_packet (int csock, BYTE *buf, int len, char *caption)
{
int     rc;                             /* Return code               */

    if (caption != NULL) {
        TNSDEBUG2("console: DBG003: Sending %s\n", caption);
        packet_trace (buf, len);
    }

    rc = send (csock, buf, len, 0);

    if (rc < 0) {
        WRMSG(HHC01034, "E", "send()", strerror(HSO_errno));
        return -1;
    } /* end if(rc) */

    return 0;

} /* end function send_packet */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO RECEIVE A DATA PACKET FROM THE CLIENT               */
/* This subroutine receives bytes from the client.  It stops when    */
/* the receive buffer is full, or when the last two bytes received   */
/* consist of the IAC character followed by a specified delimiter.   */
/* If zero bytes are received, this means the client has closed the  */
/* connection, and this is treated as an error.                      */
/* Input:                                                            */
/*      csock is the socket number                                   */
/*      buf points to area to receive data                           */
/*      reqlen is the number of bytes requested                      */
/*      delim is the delimiter character (0=no delimiter)            */
/* Output:                                                           */
/*      buf is updated with data received                            */
/*      The return value is the number of bytes received, or         */
/*      -1 if an error occurred.                                     */
/*-------------------------------------------------------------------*/
static int
recv_packet (int csock, BYTE *buf, int reqlen, BYTE delim)
{
int     rc=0;                           /* Return code               */
int     rcvlen=0;                       /* Length of data received   */

    while (rcvlen < reqlen) {

        rc = recv (csock, buf + rcvlen, reqlen - rcvlen, 0);

        if (rc < 0) {
            WRMSG(HHC01034, "E", "recv()", strerror(HSO_errno));
            return -1;
        }

        if (rc == 0) {
            TNSDEBUG1("console: DBG004: Connection closed by client\n");
            return -1;
        }

        rcvlen += rc;

        if (delim != '\0' && rcvlen >= 2
            && buf[rcvlen-2] == IAC && buf[rcvlen-1] == delim)
            break;
    }

    TNSDEBUG2("console: DBG005: Packet received length=%d\n", rcvlen);
    packet_trace (buf, rcvlen);

    return rcvlen;

} /* end function recv_packet */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO RECEIVE A PACKET AND COMPARE WITH EXPECTED VALUE    */
/*-------------------------------------------------------------------*/
static int
expect (int csock, BYTE *expected, int len, char *caption)
{
int     rc;                             /* Return code               */
BYTE    buf[512];                       /* Receive buffer            */

#if defined( OPTION_MVS_TELNET_WORKAROUND )

  /* TCP/IP for MVS returns the server sequence rather then the
     client sequence during bin negotiation.   Jan Jaeger, 19/06/00  */

  static BYTE do_bin[] = { IAC, DO, BINARY, IAC, WILL, BINARY };
  static BYTE will_bin[] = { IAC, WILL, BINARY, IAC, DO, BINARY };

#endif // defined( OPTION_MVS_TELNET_WORKAROUND )

    UNREFERENCED(caption);

    rc = recv_packet (csock, buf, len, 0);
    if (rc < 0)
        return -1;

#if defined( OPTION_MVS_TELNET_WORKAROUND )
    /* BYPASS TCP/IP FOR MVS WHICH DOES NOT COMPLY TO RFC1576 */
    if (1
        && memcmp(buf, expected, len) != 0
        && !(len == sizeof(will_bin)
        && memcmp(expected, will_bin, len) == 0
        && memcmp(buf, do_bin, len) == 0)
    )
#else
    if (memcmp(buf, expected, len) != 0)
#endif // defined( OPTION_MVS_TELNET_WORKAROUND )
    {
        TNSDEBUG2("console: DBG006: Expected %s\n", caption);
        return -1;
    }
    TNSDEBUG2("console: DBG007: Received %s\n", caption);

    return 0;

} /* end function expect */


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO NEGOTIATE TELNET PARAMETERS                         */
/* This subroutine negotiates the terminal type with the client      */
/* and uses the terminal type to determine whether the client        */
/* is to be supported as a 3270 display console or as a 1052/3215    */
/* printer-keyboard console.                                         */
/*                                                                   */
/* Valid display terminal types are "IBM-NNNN", "IBM-NNNN-M", and    */
/* "IBM-NNNN-M-E", where NNNN is 3270, 3277, 3278, 3279, 3178, 3179, */
/* or 3180, M indicates the screen size (2=25x80, 3=32x80, 4=43x80,  */
/* 5=27x132, X=determined by Read Partition Query command), and      */
/* -E is an optional suffix indicating that the terminal supports    */
/* extended attributes. Displays are negotiated into tn3270 mode.    */
/* An optional device number suffix (example: IBM-3270@01F) may      */
/* be specified to request allocation to a specific device number.   */
/* Valid 3270 printer type is "IBM-3287-1"                           */
/*                                                                   */
/* Terminal types whose first four characters are not "IBM-" are     */
/* handled as printer-keyboard consoles using telnet line mode.      */
/*                                                                   */
/* Input:                                                            */
/*      csock   Socket number for client connection                  */
/* Output:                                                           */
/*      class   D=3270 display console, K=printer-keyboard console   */
/*              P=3270 printer                                       */
/*      model   3270 model indicator (2,3,4,5,X)                     */
/*      extatr  3270 extended attributes (Y,N)                       */
/*      devn    Requested device number, or FFFF=any device number   */
/* Return value:                                                     */
/*      0=negotiation successful, -1=negotiation error               */
/*-------------------------------------------------------------------*/
static int negotiate( int csock, BYTE* class, BYTE* model, BYTE* extatr, U16* devn, char* group )
{
int    rc;                              /* Return code               */
char*  termtype;                        /* Pointer to terminal type  */
char*  s;                               /* String pointer            */
BYTE   c;                               /* Trailing character        */
U16    devnum;                          /* Requested device number   */
BYTE   buf[512];                        /* Telnet negotiation buffer */

static BYTE do_term   [] = { IAC, DO,   TERMINAL_TYPE };
static BYTE will_term [] = { IAC, WILL, TERMINAL_TYPE };
static BYTE req_type  [] = { IAC, SB,   TERMINAL_TYPE, SEND, IAC, SE };
static BYTE type_is   [] = { IAC, SB,   TERMINAL_TYPE, IS };
static BYTE do_eor    [] = { IAC, DO,   EOR, IAC, WILL, EOR };
static BYTE will_eor  [] = { IAC, WILL, EOR, IAC, DO,   EOR };
static BYTE do_bin    [] = { IAC, DO,   BINARY, IAC, WILL, BINARY };
static BYTE will_bin  [] = { IAC, WILL, BINARY, IAC, DO,   BINARY };
static BYTE wont_echo [] = { IAC, WONT, ECHO_OPTION };
static BYTE dont_echo [] = { IAC, DONT, ECHO_OPTION };
static BYTE will_naws [] = { IAC, WILL, NAWS };

    /* Perform terminal-type negotiation */
    rc = send_packet (csock, do_term, sizeof(do_term),
                        "IAC DO TERMINAL_TYPE");
    if (rc < 0) return -1;

    rc = expect (csock, will_term, sizeof(will_term),
                        "IAC WILL TERMINAL_TYPE");
    if (rc < 0) return -1;

    /* Request terminal type */
    rc = send_packet (csock, req_type, sizeof(req_type),
                        "IAC SB TERMINAL_TYPE SEND IAC SE");
    if (rc < 0) return -1;

    rc = recv_packet (csock, buf, sizeof(buf)-2, SE);
    if (rc < 0) return -1;

    /* Ignore Negotiate About Window Size */
    if (rc >= (int)sizeof(will_naws) &&
        memcmp (buf, will_naws, sizeof(will_naws)) == 0)
    {
        memmove(buf, &buf[sizeof(will_naws)], (rc - sizeof(will_naws)));
        rc -= sizeof(will_naws);
    }

    if (rc < (int)(sizeof(type_is) + 2)
        || memcmp(buf, type_is, sizeof(type_is)) != 0
        || buf[rc-2] != IAC || buf[rc-1] != SE) {
        TNSDEBUG2("console: DBG008: Expected IAC SB TERMINAL_TYPE IS\n");
        return -1;
    }
    buf[rc-2] = '\0';
    termtype = (char *)(buf + sizeof(type_is));
    TNSDEBUG2("console: DBG009: Received IAC SB TERMINAL_TYPE IS %s IAC SE\n",
            termtype);

    /* Check terminal type string for device name suffix */
    s = strchr (termtype, '@');
    if(s!=NULL)
    {
        if(strlen(s)<16)
        {
            strlcpy(group,&s[1],16);
        }
    }
    else
    {
        group[0]=0;
    }

    if (s != NULL && sscanf (s, "@%hx%c", &devnum,&c) == 1)
    {
        *devn = devnum;
        group[0]=0;
    }
    else
    {
        *devn = 0xFFFF;
    }

    /* Test for non-display terminal type */
    if (memcmp(termtype, "IBM-", 4) != 0)
    {
        if (memcmp(termtype, "ANSI", 4) == 0)
        {
            rc = send_packet (csock, wont_echo, sizeof(wont_echo),
                                "IAC WONT ECHO");
            if (rc < 0) return -1;

            rc = expect (csock, dont_echo, sizeof(dont_echo),
                                "IAC DONT ECHO");
            if (rc < 0) return -1;
        }

        /* Return printer-keyboard terminal class */
        *class = 'K';
        *model = '-';
        *extatr = '-';
        return 0;
    }

    /* Determine display terminal model */
    if (memcmp(termtype+4,"DYNAMIC",7) == 0) {
        *model = 'X';
        *extatr = 'Y';
    } else {
        if (!(memcmp(termtype+4, "3277", 4) == 0
              || memcmp(termtype+4, "3270", 4) == 0
              || memcmp(termtype+4, "3178", 4) == 0
              || memcmp(termtype+4, "3278", 4) == 0
              || memcmp(termtype+4, "3179", 4) == 0
              || memcmp(termtype+4, "3180", 4) == 0
              || memcmp(termtype+4, "3287", 4) == 0
              || memcmp(termtype+4, "3279", 4) == 0))
            return -1;

        *model = '2';
        *extatr = 'N';

        if (termtype[8]=='-') {
            if (termtype[9] < '1' || termtype[9] > '5')
                return -1;
            *model = termtype[9];
            if (memcmp(termtype+4, "328",3) == 0) *model = '2';
            if (memcmp(termtype+10, "-E", 2) == 0)
                *extatr = 'Y';
        }
    }

    /* Perform end-of-record negotiation */
    rc = send_packet (csock, do_eor, sizeof(do_eor),
                        "IAC DO EOR IAC WILL EOR");
    if (rc < 0) return -1;

    rc = expect (csock, will_eor, sizeof(will_eor),
                        "IAC WILL EOR IAC DO EOR");
    if (rc < 0) return -1;

    /* Perform binary negotiation */
    rc = send_packet (csock, do_bin, sizeof(do_bin),
                        "IAC DO BINARY IAC WILL BINARY");
    if (rc < 0) return -1;

    rc = expect (csock, will_bin, sizeof(will_bin),
                        "IAC WILL BINARY IAC DO BINARY");
    if (rc < 0) return -1;

    /* Return display terminal class */
    if (memcmp(termtype+4,"3287",4)==0) *class='P';
    else *class = 'D';
    return 0;

} /* end function negotiate */

/*-------------------------------------------------------------------*/
/* NEW CLIENT CONNECTION                                             */
/*-------------------------------------------------------------------*/
static int
connect_client (int *csockp)
/* returns 1 if 3270, else 0 */
{
int                     rc;             /* Return code               */
size_t                  len;            /* Data length               */
int                     csock;          /* Socket for conversation   */
struct sockaddr_in      client;         /* Client address structure  */
socklen_t               namelen;        /* Length of client structure*/
char                   *clientip;       /* Addr of client ip address */
U16                     devnum;         /* Requested device number   */
BYTE                    class;          /* D=3270, P=3287, K=3215/1052 */
BYTE                    model;          /* 3270 model (2,3,4,5,X)    */
BYTE                    extended;       /* Extended attributes (Y,N) */
char                    buf[256];       /* Message buffer            */
char                    conmsg[256];    /* Connection message        */
char                    devmsg[25];     /* Device message            */
char                    hostmsg[256];   /* Host ID message           */
char                    num_procs[16];  /* #of processors string     */
char                    group[16];      /* Console group             */

    /* Load the socket address from the thread parameter */
    csock = *csockp;

    /* Obtain the client's IP address */
    namelen = sizeof(client);
    rc = getpeername (csock, (struct sockaddr *)&client, &namelen);

    /* Log the client's IP address and hostname */
    clientip = strdup(inet_ntoa(client.sin_addr));

    TNSDEBUG1("console: DBG018: Received connection from %s\n",
            clientip );

    /* Negotiate telnet parameters */
    rc = negotiate (csock, &class, &model, &extended, &devnum, group);
    if (rc != 0)
    {
        close_socket (csock);
        if (clientip) free(clientip);
        return 0;
    }

    /* Build connection message for client */

    if ( cons_hostinfo.num_procs > 1 )
        snprintf( num_procs, sizeof(num_procs), "MP=%d", cons_hostinfo.num_procs );
    else
        STRLCPY( num_procs, "UP" );

    snprintf
    (
        hostmsg, sizeof(hostmsg),

        "running on %s (%s-%s.%s %s %s)"

        ,cons_hostinfo.nodename
        ,cons_hostinfo.sysname
        ,cons_hostinfo.release
        ,cons_hostinfo.version
        ,cons_hostinfo.machine
        ,num_procs
    );
    snprintf (conmsg, sizeof(conmsg),
                "Hercules version %s built on %s %s",
                VERSION, __DATE__, __TIME__);

    {
        snprintf (devmsg, sizeof(devmsg), "Connected to device %4.4X",
                  0);
    }

    WRMSG(HHC01018, "I", 0, 0, clientip, 0x3270);

    /* Send connection message to client */
    if (class != 'K')
    {
        len = snprintf (buf, sizeof(buf),
                    "\xF5\x40\x11\x40\x40\x1D\x60%s"
                    "\x11\xC1\x50\x1D\x60%s"
                    "\x11\xC2\x60\x1D\x60%s",
                    prt_host_to_guest( (BYTE*) conmsg,  (BYTE*) conmsg,  strlen( conmsg  )),
                    prt_host_to_guest( (BYTE*) hostmsg, (BYTE*) hostmsg, strlen( hostmsg )),
                    prt_host_to_guest( (BYTE*) devmsg,  (BYTE*) devmsg,  strlen( devmsg  )));

        if (len < sizeof(buf))
        {
            buf[len++] = (char) IAC;
        }
        else
        {
            ASSERT(FALSE);
        }

        if (len < sizeof(buf))
        {
            buf[len++] = (char) EOR_MARK;
        }
        else
        {
            ASSERT(FALSE);
        }
    }
    else
    {
        len = snprintf (buf, sizeof(buf), "%s\r\n%s\r\n%s\r\n",
                        conmsg, hostmsg, devmsg);
    }

    if (class != 'P')  /* do not write connection resp on 3287 */
    {
        rc = send_packet (csock, (BYTE *)buf, (int)len, "CONNECTION RESPONSE");
    }
    return (class == 'D') ? 1 : 0;   /* return 1 if 3270 */
} /* end function connect_client */


static void logdump(char *txt,DEVBLK *dev,BYTE *bfr,size_t sz)
{
    char buf[128];
    char byte[5];
    size_t i;
    if(!dev->ccwtrace)
    {
        return;
    }
    WRMSG(HHC01048,"D",LCSS_DEVNUM,txt);
    WRMSG(HHC01049,"D",LCSS_DEVNUM,txt,(u_int)sz,(u_int)sz);
    buf[0] = 0;
    for(i=0;i<sz;i++)
    {
        if(i%16==0)
        {
            if(i!=0)
            {
                WRMSG(HHC01050,"D",LCSS_DEVNUM,txt,buf);
                buf[0] = 0;
            }
            MSGBUF(buf, ": %04X:", (unsigned) i);
        }
        if(i%4==0 && i)
        {
            STRLCAT( buf, " " );
        }
        MSGBUF(byte, "%02X", bfr[i]);
        STRLCAT( buf, byte );
    }
    WRMSG(HHC01050,"D",LCSS_DEVNUM,txt,buf);
    buf[0] = 0;
    for(i=0;i<sz;i++)
    {
        if(i%16==0)
        {
            if(i!=0)
            {
                WRMSG(HHC01051,"D",LCSS_DEVNUM,buf);
                buf[0] = 0;
            }
        }
        MSGBUF(byte, "%c",(bfr[i] & 0x7f) < 0x20 ? '.' : (bfr[i] & 0x7f));
        STRLCAT( buf, byte );
    }
    WRMSG(HHC01051,"D",LCSS_DEVNUM,buf);
}

static void put_bufpool(void ** anchor, BYTE * ele) {
    void ** elep = anchor;
    for (;;) {
        if (!*elep) break;
        elep = *elep;
    }
    *elep = ele;
    *(void**)ele = 0;
}

static BYTE * get_bufpool(void ** anchor) {
    void ** elep = *anchor;
    if (elep) {
        *anchor = *elep;
    } else {
        *anchor = 0;
    }
    return (BYTE*)elep;
}

static void init_bufpool(COMMADPT *ca) {
    BYTE * areap;
    int i1;
    int numbufs = 64;
    int bufsize = ca->unitsz+16+4;
    ca->poolarea = (BYTE*)calloc (numbufs, bufsize);
    if (!ca->poolarea) {
        return;
    }
    areap = ca->poolarea;
    for (i1 = 0; i1 < numbufs; i1++) {
        put_bufpool(&ca->freeq, areap);
        areap += (bufsize);
    }
}

static void free_bufpool(COMMADPT *ca) {
    ca->sendq = 0;
    ca->freeq = 0;
    if (ca->poolarea) {
        free(ca->poolarea);
        ca->poolarea = 0;
    }
}

/*-------------------------------------------------------------------*/
/* Free all private structures and buffers                           */
/*-------------------------------------------------------------------*/
static void commadpt_clean_device(DEVBLK *dev)
{
    if(dev->commadpt!=NULL)
    {
        free(dev->commadpt);
        dev->commadpt=NULL;
        if(dev->ccwtrace)
        {
            WRMSG(HHC01052,"D",
                SSID_TO_LCSS(dev->ssid),
                dev->devnum,"control block freed");
        }
    }
    else
    {
        if(dev->ccwtrace)
        {
            WRMSG(HHC01052,"D",
                SSID_TO_LCSS(dev->ssid),
                dev->devnum,"control block not freed: not allocated");
        }
    }
    return;
}

/*-------------------------------------------------------------------*/
/* Allocate initial private structures                               */
/*-------------------------------------------------------------------*/
static int commadpt_alloc_device(DEVBLK *dev)
{
    dev->commadpt=malloc(sizeof(COMMADPT));
    if(dev->commadpt==NULL)
    {
        char buf[40];
        MSGBUF(buf, "malloc(%d)", (int)sizeof(COMMADPT));
        WRMSG(HHC01000, "E", LCSS_DEVNUM, buf, strerror(errno));
        return -1;
    }
    memset(dev->commadpt,0,sizeof(COMMADPT));
    dev->commadpt->dev=dev;
    return 0;
}
/*-------------------------------------------------------------------*/
/* Parsing utilities                                                 */
/*-------------------------------------------------------------------*/
/*-------------------------------------------------------------------*/
/* commadpt_getport : returns a port number or -1                    */
/*-------------------------------------------------------------------*/
static int commadpt_getport(char *txt)
{
    int pno;
    struct servent *se;
    pno=atoi(txt);
    if(pno==0)
    {
        se=getservbyname(txt,"tcp");
        if(se==NULL)
        {
            return -1;
        }
        pno=se->s_port;
    }
    return(pno);
}
/*-------------------------------------------------------------------*/
/* commadpt_getaddr : set an in_addr_t if ok, else return -1         */
/*-------------------------------------------------------------------*/
static int commadpt_getaddr(in_addr_t *ia,char *txt)
{
    struct hostent *he;
    he=gethostbyname(txt);
    if(he==NULL)
    {
        return(-1);
    }
    memcpy(ia,he->h_addr_list[0],4);
    return(0);
}

static void connect_message(int sfd, int na, int flag) {
    struct sockaddr_in client;
    socklen_t namelen;
    char *ipaddr;
    char msgtext[256];
    if (!sfd)
        return;
    namelen = sizeof(client);
    (void)getpeername (sfd, (struct sockaddr *)&client, &namelen);
    ipaddr = inet_ntoa(client.sin_addr);
    if (flag == 0)
        MSGBUF( msgtext, "%s:%d VTAM CONNECTION ACCEPTED - NETWORK NODE= %4.4X",
                ipaddr, (int)ntohs(client.sin_port), na);
    else
        MSGBUF( msgtext, "%s:%d VTAM CONNECTION TERMINATED",
                ipaddr, (int)ntohs(client.sin_port));
    WRMSG( HHC01047, "I", msgtext );
    VERIFY(0 <= write(sfd, msgtext, (u_int)strlen(msgtext)));
    VERIFY(2 == write(sfd, "\r\n", 2));
}

static void commadpt_read_tty(COMMADPT *ca, BYTE * bfr, int len)
// everything has been tortured to now do 3270 also
{
    BYTE        bfr3[3];
    BYTE        c;
    int i1;
    int eor=0;
    logdump("RECV",ca->dev, bfr,len);
    /* If there is a complete data record already in the buffer
       then discard it before reading more data
       For TTY, allow data to accumulate until CR is received */
    if (ca->is_3270) {
            if (ca->inpbufl) {
                ca->rlen3270 = 0;
                ca->inpbufl = 0;
            }
        }
    for (i1 = 0; i1 < len; i1++) {
        c = (unsigned char) bfr[i1];
        if (ca->telnet_opt) {
            ca->telnet_opt = 0;
            if(ca->dev->ccwtrace)
                WRMSG(HHC01053,"D",
                    SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum,
                    ca->telnet_cmd, c);
            bfr3[0] = 0xff;  /* IAC */
            /* set won't/don't for all received commands */
            bfr3[1] = (ca->telnet_cmd == 0xfd) ? 0xfc : 0xfe;
            bfr3[2] = c;
            if (ca->sfd > 0) {
                write_socket(ca->sfd,bfr3,3);
            }
            if(ca->dev->ccwtrace)
                WRMSG(HHC01054,"D",
                    SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum,
                    bfr3[1], bfr3[2]);
            continue;
        }
        if (ca->telnet_iac) {
            ca->telnet_iac = 0;
            if(ca->dev->ccwtrace)
                WRMSG(HHC01055, "D",
                    SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum,
                    c);
            switch (c) {
            case 0xFB:  /* TELNET WILL option cmd */
            case 0xFD:  /* TELNET DO option cmd */
                ca->telnet_opt = 1;
                ca->telnet_cmd = c;
                break;
            case 0xF4:  /* TELNET interrupt */
                if (!ca->telnet_int) {
                    ca->telnet_int = 1;
                }
                break;
            case EOR_MARK:
                                eor = 1;
                break;
            case 0xFF:  /* IAC IAC */
                        ca->inpbuf[ca->rlen3270++] = 0xFF;
                break;
            }
            continue;
        }
        if (c == 0xFF) {  /* TELNET IAC */
            ca->telnet_iac = 1;
            continue;
        } else {
            ca->telnet_iac = 0;
        }
        if (!ca->is_3270) {
            if (c == 0x0D) // CR in TTY mode ?
                ca->eol_flag = 1;
            c = host_to_guest(c);   // translate ASCII to EBCDIC for tty
        }
        ca->inpbuf[ca->rlen3270++] = c;
    }
    /* received data (rlen3270 > 0) is sufficient for 3270,
       but for TTY, eol_flag must also be set */
    if ((ca->eol_flag || ca->is_3270) && ca->rlen3270)
    {
        ca->eol_flag = 0;
        if (ca->is_3270)
        {
            if (eor)
            {
                ca->inpbufl = ca->rlen3270;
                ca->rlen3270 = 0; /* for next msg */
            }
        }
        else
        {
            ca->inpbufl = ca->rlen3270;
            ca->rlen3270 = 0; /* for next msg */
        }
        if(ca->dev->ccwtrace)
            WRMSG(HHC01056, "D",
                SSID_TO_LCSS(ca->dev->ssid),
                ca->dev->devnum,
                ca->inpbufl);
    }
}

static void *telnet_thread(void *vca)
{
    COMMADPT *ca;
    int devnum;                 /* device number copy for convenience*/
    int sockopt;                /* Used for setsocketoption          */
    int rc;                     /* return code from various rtns     */
    struct sockaddr_in sin;     /* bind socket address structure     */
    BYTE bfr[256];
    U32 stids;

    ca=(COMMADPT*)vca;
    /* get a work copy of devnum (for messages) */
    ca->sfd = 0;
    devnum=ca->devnum;

    ca->lfd=socket(AF_INET,SOCK_STREAM,0);
    if(!socket_is_socket(ca->lfd))
    {
        WRMSG(HHC01002, "E",SSID_TO_LCSS(ca->dev->ssid),devnum,strerror(HSO_errno));
        ca->have_cthread=0;
        release_lock(&ca->lock);
        return NULL;
    }

    /* Reuse the address regardless of any */
    /* spurious connection on that port    */
    sockopt=1;
    setsockopt(ca->lfd,SOL_SOCKET,SO_REUSEADDR,(GETSET_SOCKOPT_T*)&sockopt,sizeof(sockopt));

    /* Bind the socket */
    sin.sin_family=AF_INET;
    sin.sin_addr.s_addr=ca->lhost;
    sin.sin_port=htons(ca->lport);
    rc=bind(ca->lfd,(struct sockaddr *)&sin,sizeof(sin));
    if (rc < 0)
    {
        WRMSG(HHC01000, "E",SSID_TO_LCSS(ca->dev->ssid),devnum,"bind()",strerror(HSO_errno));
        return NULL;
    }
    /* Start the listen */
    rc = listen(ca->lfd,10);
    if (rc < 0)
    {
        WRMSG(HHC01000, "E",SSID_TO_LCSS(ca->dev->ssid),devnum,"listen()",strerror(HSO_errno));
        return NULL;
    }
    WRMSG(HHC01004, "I", SSID_TO_LCSS(ca->dev->ssid), devnum, ca->lport);
    for (;;)
    {
        ca->sfd = 0;
        ca->sfd=accept(ca->lfd,NULL,0);
        if (ca->sfd < 1)
            continue;
        if  (connect_client(&ca->sfd))
        {
            ca->is_3270 = 1;
        }
        else
        {
            ca->is_3270 = 0;
        }
        socket_set_blocking_mode(ca->sfd,0);  // set to non-blocking mode
        stids = ((ca->idblk << 20) & 0xfff00000) | (ca->idnum & 0x000fffff); // 12 bit IDBLK, 20 bit IDNUM
        if (ca->emu3791 == 0) {sna_reqcont(ca, (ca->is_3270) ? 0x02 : 0x01, stids);}   // send REQCONT
        ca->hangup = 0;
        for (;;)
        {
            usleep(50000);
            if (ca->hangup)
                break;
            /* read_socket has changed from 3.04 to 3.06 - we need old way */
#ifdef _MSVC_
            rc=recv(ca->sfd,bfr,ca->unitsz-BUFPD,0);
#else
            rc=read(ca->sfd,bfr,ca->unitsz-BUFPD);
#endif
            if (rc < 0)
            {
                if (0
#ifndef WIN32
                    || EAGAIN == errno
#endif
                    || HSO_EWOULDBLOCK == HSO_errno
                )
                {
                    continue;
                }
                break;
            }
            if (rc == 0)
            {
//              sna_reqcont(ca, 1);   // send REQDISCONT
                if (ca->emu3791 == 0) {sna_inop(ca);}
                break;
            }
            commadpt_read_tty(ca,bfr,rc);
        }
        close_socket(ca->sfd);
        ca->sfd = 0;
    }

    UNREACHABLE_CODE( return NULL );
}

void dlsw_inforeq(COMMADPT *ca, BYTE *requestp, BYTE laddr)
{
    int amt, rc;
    BYTE buf[1024];

    memcpy(buf, ca->pkt, 16);

    if ((requestp[6] == 0) && (requestp[7] == 0))       /* sequence number = 0? */
    {
        return;                                         /* yes, don't send */
    }
#if 0
    if (requestp[13] == 0x31)                           /* BIND */
    {
        if (requestp[22] == 0)                          /* no SRCVPAC? */
        {
            requestp[22] = 4;                           /* default to 4 */
        }

        ca->session = 1;                                /* set session active */
        ca->pacing = requestp[22];                      /* save pacing value */
        ca->pwindow = ca->pacing;                       /* set initial window */
    }
    else if (requestp[13] == 0x32)                      /* UNBIND */
    {
        ca->session = 0;                                /* clear session */
    }

    if (ca->session)                                    /* if session active */
    {
        if ((requestp[0] & 0x1) == 0)                   /* EFI not set */
        {
            if (ca->pwindow == 0)
            {
                DLSW_DEBUG ("Pacing window reached 0\n");
                //queue request
                //return;
            }
            if ((ca->pwindow % ca->pacing) == 0)
            {
                requestp[11] |= 0x1;                    /* set pacing indicator */
            }
            ca->pwindow--;
            DLSW_DEBUG("Window is now at %d\n", ca->pwindow);
        }
    }
#endif
    th_remap(MAP_FID1_FID2, requestp, ca->locsuba);
    requestp[6] = laddr;                                /* DAF */
//    requestp[7] = 1;                                    /* OAF */
    amt = (requestp[0] << 8) + requestp[1];
    amt -= 4;
    memcpy(&buf[LEN_INFO], &requestp[4], amt);

    buf[HDR_HLEN] = LEN_INFO;
    buf[HDR_MTYP] = INFOFRAME;
    PUT16(buf, HDR_MLEN, htons(amt));
//    buf[HDR_DIR] = DIR_ORG;
    PUT32(buf, HDR_RDLC, ca->dlc);
    PUT32(buf, HDR_RDPID, ca->dlc_pid);
    DLSW_DEBUG("--> PRE_INFOFRAME\n");
    rc = write_socket(ca->wfd, buf, LEN_INFO + amt);
    if (rc < 0)
        DLSW_DEBUG("ERROR writing to socket");
    DLSW_DEBUG("--> INFOFRAME\n");
}

void dlsw_inforesp(COMMADPT *ca, BYTE *buf)
{
    BYTE    *respbuf;
    void    *eleptr;
    int     amt, i;
    BYTE    daf, oaf;
    int     found = 0;

    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
            WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA request");
            return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    amt = ntohs(GET16(buf, HDR_MLEN));
    memcpy(&respbuf[4], &buf[LEN_INFO], amt);
    amt += 4;
    respbuf[0] = (amt >> 8) & 0xff;
    respbuf[1] = amt & 0xff;
    daf = respbuf[6];
    oaf = respbuf[7];

    th_remap(MAP_FID2_FID1, respbuf, ca->locsuba);

    for (i = 0; i < 20; i++)
    {
        if ((ca->dlsw_map[i].valid) && (ca->dlsw_map[i].laddr == oaf))
        {
            respbuf[4] = ca->dlsw_map[i].addr0;
            respbuf[5] = ca->dlsw_map[i].addr1;
            found = 1;
            break;
        }
    }
    if (found == 0)
    {
        logmsg ("WARNING: DLSw response packet lost, oaf = 0x%x\n", oaf);
    }

    respbuf[2] = ca->sscp_addr0;
    respbuf[3] = ca->sscp_addr1 + daf;
#if 0
    if (respbuf[13] == 0x31)                            /* BIND */
    {
        if (respbuf[22] == 0)                           /* no SRCVPAC? */
        {
            respbuf[22] = 4;                            /* default to 4 */
        }

        ca->session = 1;                                /* set session active */
        ca->pacing = respbuf[22];                       /* save pacing value */
        ca->pwindow = ca->pacing;                       /* set initial window */
    }
    else if (respbuf[13] == 0x32)                       /* UNBIND */
    {
        ca->session = 0;                                /* clear session */
    }

    if (ca->session)                                    /* if session active */
    {
        if (respbuf[11] & 0x1)                          /* PI set? */
        {
            ca->pwindow = ca->pwindow + ca->pacing;
            DLSW_DEBUG("Window is now at %d\n", ca->pwindow);
            //release queued requests
        }
    }
#endif
    put_bufpool(&ca->sendq, eleptr);
}

void dlsw_contact(COMMADPT *ca, BYTE *requestp)
{
    int n;
    BYTE buf[1024];

    memcpy(ca->req, requestp, 18);
    memcpy(buf, ca->pkt, 1024);

    buf[HDR_MTYP] = CONTACT;
    PUT16(buf, HDR_MLEN, 0);
    buf[HDR_DIR] = DIR_ORG;
    PUT32(buf, HDR_RDLC, ca->dlc);
    PUT32(buf, HDR_RDPID, ca->dlc_pid);
    n = write_socket(ca->wfd, buf, LEN_CTRL);
    if (n < 0)
        DLSW_DEBUG("ERROR writing to socket");
    DLSW_DEBUG("--> CONTACT\n");
}

void dlsw_contacted(COMMADPT *ca)
{
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;

    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
            WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA request");
            return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    /* first do the ten-byte FID1 TH */
    respbuf[0] = 0x1c;
    respbuf[1] = 0x00;
    respbuf[2] = ca->req[4];   // daf
    respbuf[3] = ca->req[5];
    respbuf[4] = ca->req[2];   // oaf
    respbuf[5] = ca->req[3];
    make_seq(ca, respbuf);
    /* do RH */
    respbuf[10] = ca->req[10];
    respbuf[11] = ca->req[11];
    respbuf[11] = 0x00;
    respbuf[12] = ca->req[12];

    /* make a CONTACTED RU */
    ru_size = 0;
    ru_ptr = &respbuf[13];
    ru_ptr[ru_size++] = 0x01;
    ru_ptr[ru_size++] = 0x02;
    ru_ptr[ru_size++] = 0x80;
    ru_ptr[ru_size++] = ca->req[16];
    ru_ptr[ru_size++] = ca->req[17];
    ru_ptr[ru_size++] = 0x01;

    /* set length field in TH */
    ru_size += 3;   /* for RH */
    respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
    respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

    put_bufpool(&ca->sendq, eleptr);

    ca->circuit = 1;
}

int send_capabilities(COMMADPT *ca, unsigned char *buf)
{
    unsigned int off = LEN_CTRL + 4;
    int rc;

    ca->flow_control = 0;

    buf[off++] = 0x05;                                  /* Vendor ID */
    buf[off++] = CAP_VID;
    buf[off++] = 0x00;
    buf[off++] = 0x00;
    buf[off++] = 0x00;

    buf[off++] = 0x04;                                  /* DLSw Version */
    buf[off++] = CAP_VER;
    buf[off++] = 0x02;
    buf[off++] = 0x00;

    buf[off++] = 0x04;                                  /* Initial Pacing Window */
    buf[off++] = CAP_IPW;
    buf[off++] = 0x00;
    buf[off++] = 0x14;

    buf[off++] = 0x12;                                  /* Supported SAP List */
    buf[off++] = CAP_SSL;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;
    buf[off++] = 0xFF;

    buf[off++] = 0x03;                                  /* TCP Connections */
    buf[off++] = CAP_TCP;
    buf[off++] = 0x02;                                  /* use two connections */

    PUT16(buf, LEN_CTRL, htons(off - LEN_CTRL));
    PUT16(buf, LEN_CTRL + 2, htons(0x1520));

    buf[HDR_VER] = DLSW_VER;
    buf[HDR_HLEN] = LEN_CTRL;
    PUT16(buf, HDR_MLEN, htons(off - LEN_CTRL));
    buf[HDR_MTYP] = CAP_EXCHANGE;
    buf[HDR_PID] = 0x42;
    buf[HDR_NUM] = 0x01;
    buf[HDR_DIR] = DIR_TGT;

    rc = write_socket(ca->wfd, buf, off);
    if (rc < 0)
        return rc;
    DLSW_DEBUG("--> CAP_EXCHANGE\n");
    return 0;
}

int process_capabilities(COMMADPT *ca, unsigned char *buf)
{
    int close_read = 0;
    int close_write = 0;
    unsigned int msg_off = LEN_CTRL;
    unsigned int cap_len = ntohs(GET16(buf, msg_off));
    unsigned int gds_id = ntohs(GET16(buf, msg_off + 2));
    unsigned int off = msg_off + 4;
    int rc;
    unsigned int len, typ;

    if (gds_id == 0x1521)
    {
        DLSW_DEBUG("capabilities response\n");
        return 0;
    }
    else if (gds_id != 0x1520)
    {
        DLSW_DEBUG("unknown capabilities exchange\n");
        return 0;
    }

    DLSW_DEBUG("cap_len = %d\n", cap_len);
    while (off < (cap_len + msg_off))
    {
        len = buf[off];
        typ = buf[off+1];
        DLSW_DEBUG("CAP: offset = %d, len = %d\n", off, len);
        switch (typ)
        {
            case CAP_VID:                               /* Vendor ID */
                DLSW_DEBUG("CAP: Vendor ID\n");
                break;

            case CAP_VER:                               /* DLSw Version */
                DLSW_DEBUG("CAP: DLSw Version\n");
                break;

            case CAP_IPW:                               /* Initial Pacing Window */
                DLSW_DEBUG("CAP: Initial Pacing Window\n");
                ca->init_window = ntohs(GET16(buf, off));
                ca->window = ca->init_window;
                ca->granted = 0;
                ca->fca_owed = 0;
                break;

            case CAP_VERS:                              /* Version String */
                DLSW_DEBUG("CAP: Version String\n");
                break;

            case CAP_MACX:                              /* MAC Address Exclusivity */
                DLSW_DEBUG("CAP: MAC Address Exclusivity\n");
                break;

            case CAP_SSL:                               /* Supported SAP List */
                DLSW_DEBUG("CAP: Supported SAP List\n");
                break;

            case CAP_TCP:                               /* TCP Connections */
                DLSW_DEBUG("CAP: TCP Connection\n");
                /* The following code has been disabled to keep both connections open */
#if 0
                if ((buf[off+2] == 1) && (ca->rfd != ca->wfd))
                {
                    if (ca->high_ip)
                    {
                        close_read = 1;
                    }
                    else
                    {
                        close_write = 1;
                    }
                }
#endif
                break;

            case CAP_NBX:                               /* NetBIOS Name Exclusivity */
                DLSW_DEBUG("CAP: NetBIOS Name Exclusivity\n");
                break;

            case CAP_MACL:                              /* MAC Address List */
                DLSW_DEBUG("CAP: MAC Address List\n");
                break;

            case CAP_NBL:                               /* NetBIOS Name List */
                DLSW_DEBUG("CAP: NetBIOS Name List\n");
                break;

            case CAP_VC:                                /* Vendor Context */
                DLSW_DEBUG("CAP: Vendor Context\n");
                break;

            default:
                DLSW_DEBUG("CAP: Unknown 0x%02X\n", typ);
                break;
        }
        off = off + len;
    }

    PUT16(buf, HDR_MLEN, htons(0x0004));                /* Message Length */
    PUT16(buf, msg_off, htons(0x0004));                 /* GDS Length */
    PUT16(buf, msg_off + 2, htons(0x1521));             /* GDS ID = Capabilities Response */

    rc = write_socket(ca->wfd, buf, msg_off + 4);              /* send response */
    if (rc < 0)
        return rc;
    DLSW_DEBUG("--> CAP_EXCHANGE(r)\n");

    if (close_read)
    {
        DLSW_DEBUG("Closing read socket\n");
        close_socket(ca->rfd);
        ca->rfd = ca->wfd;
    }
    else if (close_write)
    {
        DLSW_DEBUG("Closing write socket\n");
        ca->wfd = ca->rfd;
    }
    else
    {
        DLSW_DEBUG("Using two connections\n");
    }
    return 0;
}

void process_flow_control(COMMADPT *ca, unsigned char *buf)
{
    if (buf[HDR_FCB] & FCB_FCI)
    {
        switch (buf[HDR_FCB] & FCB_FCO)
        {
            case FCO_RPT:
                ca->granted += ca->window;
                break;

            case FCO_INC:
                ca->window++;
                ca->granted += ca->window;
                break;

            case FCO_DEC:
                ca->window--;
                ca->granted += ca->window;
                break;

            case FCO_RST:
                ca->window = 0;
                ca->granted = 0;
                break;

            case FCO_HLV:
                if (ca->window > 1)
                {
                    ca->window = ca->window / 2;
                }
                ca->granted += ca->window;
                break;
        }
    }
    if (buf[HDR_FCB] & FCB_FCA)
    {
        ca->fca_owed = 0;
    }
    if (ca->fca_owed)
    {
        buf[HDR_FCB] = 0;
    }
    else
    {
        buf[HDR_FCB] = FCB_FCI | FCO_RPT;
        ca->fca_owed = 1;
    }
}

int process_packet(COMMADPT *ca, unsigned char *buf)
{
    BYTE msg_type = buf[HDR_MTYP];
    U16 msg_len = ntohs(GET16(buf, HDR_MLEN));
    U32 stids;
    int rc;

    if (ca->flow_control)
    {
        process_flow_control(ca, buf);
    }

    switch (msg_type)
    {
        case CANUREACH:                                 /* Can U Reach Station */
            if (buf[HDR_SFLG] & SSPex)
                DLSW_DEBUG("<-- CANUREACH(ex)\n");
            else
            {
                DLSW_DEBUG("<-- CANUREACH(cs)\n");
                ca->dlc = GET32(buf, HDR_ODLC);
                ca->dlc_pid = GET32(buf, HDR_ODPID);
            }
            PUT16(buf, HDR_MLEN, 0);
            buf[HDR_MTYP] = ICANREACH;
            buf[HDR_DIR] = DIR_ORG;
            PUT32(buf, HDR_RDLC, GET32(buf, HDR_ODLC));
            PUT32(buf, HDR_RDPID, GET32(buf, HDR_ODPID));
            rc = write_socket(ca->wfd, buf, LEN_CTRL);
            if (rc < 0)
                return rc;
            if (buf[HDR_SFLG] & SSPex)
                DLSW_DEBUG("--> ICANREACH(ex)\n");
            else
                DLSW_DEBUG("--> ICANREACH(cs)\n");
            break;

        case REACH_ACK:
            DLSW_DEBUG("<-- REACH_ACK\n");
            ca->flow_control = 1;
            break;

        case XIDFRAME:
            DLSW_DEBUG("<-- XIDFRAME\n");
            if (msg_len > 0)                            /* received XID? */
            {
                memcpy(ca->pkt, buf, 1024);

                stids = ntohl(GET32(buf, LEN_CTRL+2));
                sna_reqcont(ca, buf[LEN_CTRL] & 0xf, stids);
                DLSW_DEBUG("--> REQCONT\n");
            }
            else                                        /* no, NULL XID */
            {
                PUT16(buf, HDR_MLEN, htons(20));
                buf[HDR_DIR] = DIR_ORG;
                PUT32(buf, HDR_RDLC, GET32(buf, HDR_ODLC));
                PUT32(buf, HDR_RDPID, GET32(buf, HDR_ODPID));
                buf[LEN_CTRL+0] = 0x14;
                buf[LEN_CTRL+1] = 0x01;
                buf[LEN_CTRL+2] = 0;
                buf[LEN_CTRL+3] = 0;
                buf[LEN_CTRL+4] = 0;
                buf[LEN_CTRL+5] = 0;
                buf[LEN_CTRL+6] = 0;
                buf[LEN_CTRL+7] = 0;
                buf[LEN_CTRL+8] = 0;
                buf[LEN_CTRL+9] = 0;
                buf[LEN_CTRL+10] = 0;
                buf[LEN_CTRL+11] = 0;
                buf[LEN_CTRL+12] = 0;
                buf[LEN_CTRL+13] = 0;
                buf[LEN_CTRL+14] = 0;
                buf[LEN_CTRL+15] = 0;
                buf[LEN_CTRL+16] = 0;
                buf[LEN_CTRL+17] = 0;
                buf[LEN_CTRL+18] = 0;
                buf[LEN_CTRL+19] = 0;
                rc = write_socket(ca->wfd, buf, LEN_CTRL+20);
                if (rc < 0)
                    return rc;
                DLSW_DEBUG("--> XIDFRAME\n");
            }
            break;

#if 0
        case CONTACT:
            DLSW_DEBUG("<-- CONTACT\n");
            PUT16(buf, HDR_MLEN, 0);
            buf[HDR_MTYP] = CONTACTED;
            buf[HDR_DIR] = DIR_ORG;
            PUT32(buf, HDR_RDLC, GET32(buf, HDR_ODLC));
            PUT32(buf, HDR_RDPID, GET32(buf, HDR_ODPID));
            rc = write_socket(ca->wfd, buf, LEN_CTRL);
            if (rc < 0)
                return rc;
            DLSW_DEBUG("--> CONTACTED\n");
            break;
#endif

        case CONTACTED:
            DLSW_DEBUG("<-- CONTACTED\n");
            dlsw_contacted(ca);
            break;

        case INFOFRAME:
            DLSW_DEBUG("<-- INFOFRAME\n");
            dlsw_inforesp(ca, buf);
            break;

        case HALT_DL:
            DLSW_DEBUG("<-- HALT_DL\n");
            if (ca->circuit)
            {
                // INOP
                sna_inop(ca);
                ca->circuit = 0;
            }
            PUT16(buf, HDR_MLEN, 0);
            buf[HDR_MTYP] = DL_HALTED;
            buf[HDR_DIR] = DIR_ORG;
            PUT32(buf, HDR_RDLC, GET32(buf, HDR_ODLC));
            PUT32(buf, HDR_RDPID, GET32(buf, HDR_ODPID));
            rc = write_socket(ca->wfd, buf, LEN_CTRL);
            if (rc < 0)
                return rc;
            DLSW_DEBUG("--> DL_HALTED\n");
            break;

        case RESTART_DL:
            DLSW_DEBUG("<-- RESTART_DL\n");
            PUT16(buf, HDR_MLEN, 0);
            buf[HDR_MTYP] = DL_RESTARTED;
            buf[HDR_DIR] = DIR_ORG;
            PUT32(buf, HDR_RDLC, GET32(buf, HDR_ODLC));
            PUT32(buf, HDR_RDPID, GET32(buf, HDR_ODPID));
            rc = write_socket(ca->wfd, buf, LEN_CTRL);
            if (rc < 0)
                return rc;
            DLSW_DEBUG("--> DL_RESTARTED\n");
            break;

        case CAP_EXCHANGE:                              /* Capabilities Exchange */
            DLSW_DEBUG("<-- CAP_EXCHANGE\n");
            return process_capabilities(ca, buf);
    }
    return 0;
}

static void *dlsw_thread(void *vca)
{
    COMMADPT *ca;
    int devnum;                 /* device number copy for convenience*/
    int sockopt;                /* Used for setsocketoption          */
    int rc;                     /* return code from various rtns     */
    struct sockaddr_in sin;     /* bind socket address structure     */
    struct sockaddr_in clientaddr; /* client addr */
    struct sockaddr_in serveraddr; /* server addr */
    unsigned int clientlen;     /* byte size of client's address */
    unsigned int serverlen;     /* byte size of server's address */
    char *clientip;             /* Addr of client ip address */
    int rcv_size;
    int exp_size;
    int rcv_hdr;
    BYTE buf[1024];
    int i;

    ca = (COMMADPT*)vca;
    /* get a work copy of devnum (for messages) */
    ca->sfd = 0;
    devnum = ca->devnum;

    DLSW_DEBUG("DLSw thread starting for dev %d\n", devnum);

    for (i = 0; i < 20; i++)
    {
        ca->dlsw_map[i].valid = 0;
    }

    DLSW_DEBUG("DLSw: create server socket\n");

    ca->lfd = socket(AF_INET, SOCK_STREAM, 0);
    if (!socket_is_socket(ca->lfd))
    {
        WRMSG(HHC01002, "E", SSID_TO_LCSS(ca->dev->ssid), devnum, strerror(HSO_errno));
        ca->have_cthread = 0;
        release_lock(&ca->lock);
        return NULL;
    }

    DLSW_DEBUG("DLSw: setsocketopt()\n");

    /* Reuse the address regardless of any */
    /* spurious connection on that port    */
    sockopt = 1;
    setsockopt(ca->lfd, SOL_SOCKET, SO_REUSEADDR, (GETSET_SOCKOPT_T*)&sockopt, sizeof(sockopt));

    /* Bind the socket */
    sin.sin_family = AF_INET;
    sin.sin_addr.s_addr = ca->lhost;
    sin.sin_port = htons(ca->lport);

    DLSW_DEBUG("DLSw: bind()\n");

    rc = bind(ca->lfd, (struct sockaddr *)&sin, sizeof(sin));
    if (rc < 0)
    {
        WRMSG(HHC01000, "E", SSID_TO_LCSS(ca->dev->ssid), devnum, "bind()", strerror(HSO_errno));
    }
    else
    {
        DLSW_DEBUG("DLSw: listen()\n");
        listen(ca->lfd, 10);
        WRMSG(HHC01004, "I", SSID_TO_LCSS(ca->dev->ssid), devnum, ca->lport);
    }

    for (;;)
    {
        ca->rfd = 0;
        clientlen = sizeof(clientaddr);
        DLSW_DEBUG("DLSw: accept()\n");
        ca->rfd = accept(ca->lfd, (struct sockaddr *) &clientaddr, &clientlen);
        if (ca->rfd < 1) continue;
#if 0
        socket_set_blocking_mode(ca->rfd, 0);  // set to non-blocking mode
#endif

        DLSW_DEBUG("DLSw: inet_ntoa()\n");

        /*
         * determine who sent the message
         */
        clientip = inet_ntoa(clientaddr.sin_addr);
        if (clientip == NULL)
        {
            WRMSG(HHC01035, "E", "inet_ntoa()", strerror(HSO_errno));
            close_socket(ca->rfd);
            continue;
        }
        ca->rhost = clientaddr.sin_addr.s_addr;
        WRMSG(HHC01018, "I", 0, 0, clientip, 0x3174);

        DLSW_DEBUG("DLSw: getsockname()\n");

        /*
         * determine the local address
         */
        serverlen = sizeof(serveraddr);
        getsockname(ca->rfd, (struct sockaddr *) &serveraddr, &serverlen);

        /*
         * determine if we have the higher ip address
         */
        DLSW_DEBUG ("server = %08x, client = %08x\n", serveraddr.sin_addr.s_addr, clientaddr.sin_addr.s_addr);
        if (serveraddr.sin_addr.s_addr > clientaddr.sin_addr.s_addr)
        {
            ca->high_ip = 1;
        }
        else
        {
            ca->high_ip = 0;
        }

        DLSW_DEBUG("DLSw: create client socket\n");

        /*
         * setup reverse connection
         */
        ca->wfd = socket(AF_INET, SOCK_STREAM, 0);
        if (!socket_is_socket(ca->wfd))
        {
            WRMSG(HHC01000, "E", SSID_TO_LCSS(ca->dev->ssid), devnum, "socket()", strerror(HSO_errno));
            close_socket(ca->rfd);
            continue;
        }

        serveraddr.sin_family = AF_INET;
        serveraddr.sin_addr.s_addr = ca->rhost;
        serveraddr.sin_port = htons(ca->rport);

        DLSW_DEBUG("DLSw: connect()\n");

        if (connect(ca->wfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0)
        {
            WRMSG(HHC01000, "E", SSID_TO_LCSS(ca->dev->ssid), devnum, "connect()", strerror(HSO_errno));
            close_socket(ca->wfd);
            close_socket(ca->rfd);
            continue;
        }

        if (disable_nagle(ca->wfd) < 0)
        {
            DLSW_DEBUG("DLSw: Failed to set nodelay for wfd\n");
        }

        if (disable_nagle(ca->rfd) < 0)
        {
            DLSW_DEBUG("DLSw: Failed to set nodelay for rfd\n");
        }

        DLSW_DEBUG("DLSw: send_capabilities()\n");

        /*
         * send capabilities to the server
         */
        memset(buf, 0, 1024);
        rc = send_capabilities(ca, buf);

        if (rc != 0)
            DLSW_DEBUG("DLSw: send_capabilities() rc = %d, errno = %d\n", rc, errno);

        ca->hangup = 0;
        exp_size = LEN_INFO;
        rcv_hdr = 1;
        rcv_size = 0;
        for (;;)
        {
            while (rcv_size < exp_size)
            {
                usleep(50000);
                if (ca->hangup)
                    break;

#ifdef _MSVC_
                rc = recv(ca->rfd, &buf[rcv_size], exp_size-rcv_size, 0);
#else
                rc = read(ca->rfd, &buf[rcv_size], exp_size-rcv_size);
#endif
                if (rc < 0)
                {
                    if (0
#ifndef WIN32
                                || EAGAIN == errno
#endif
                                || HSO_EWOULDBLOCK == HSO_errno
                    )
                    {
                        continue;
                    }
                    break;
                }
                if (rc == 0)
                {
                    break;
                }

                rcv_size += rc;
            }

            if (rc <= 0)
                break;

            if (rcv_hdr)
            {
                /*
                 * calculate remaining packet size
                 */
                exp_size = buf[HDR_HLEN];
                exp_size = exp_size + ntohs(GET16(buf, HDR_MLEN));
                rcv_hdr = 0;
            }
            else
            {
                rc = process_packet(ca, buf);
                if (rc < 0)
                    break;
                exp_size = LEN_INFO;
                rcv_hdr = 1;
                rcv_size = 0;
            }
        }
        WRMSG(HHC01022, "I", 0, 0, clientip, 0x3174);
        DLSW_DEBUG("Client disconnected, rc = %d, errno = %d\n", rc, errno);
        close_socket(ca->rfd);
        close_socket(ca->wfd);
        if (ca->circuit)
        {
            // INOP
            sna_inop(ca);
        }
        ca->rfd = 0;
        ca->wfd = 0;
        ca->circuit = 0;
        ca->session = 0;
        for (i = 0; i < 20; i++)
        {
            ca->dlsw_map[i].valid = 0;
        }
    }
}

/*-------------------------------------------------------------------*/
/* Communication Thread main loop                                    */
/*-------------------------------------------------------------------*/
static void *commadpt_thread(void *vca)
{
    COMMADPT    *ca;            /* Work CA Control Block Pointer     */
    int devnum;                 /* device number copy for convenience*/
    int delay;                  /* unacknowledged attention delay    */
    int rc;                     /* return code from various rtns     */
    char threadname[40];        /* string for WRMSG               */

    /*---------------------END OF DECLARES---------------------------*/

    /* fetch the commadpt structure */
    ca=(COMMADPT *)vca;

    /* Obtain the CA lock */
    obtain_lock(&ca->lock);

    /* get a work copy of devnum (for messages) */
    devnum=ca->devnum;

    MSGBUF(threadname, "3705 device(%1d:%04X) thread", ca->dev->ssid, devnum);
    LOG_THREAD_BEGIN( threadname );

    while (!sysblk.shutdown) {
        release_lock(&ca->lock);
        if(ca->ackspeed == 0) delay = 50000 + (ca->unack_attn_count * 100000);         /* Max's reliable algorithm      */
        else delay = (ca->unack_attn_count * ca->unack_attn_count + 1) * ca->ackspeed; /* much faster but TCAM hates it */
        usleep(min(1000000,delay));                                                    /* go to sleep, max. 1 second    */
        obtain_lock(&ca->lock);
        make_sna_requests2(ca);
        sna_sig(ca);
        if (ca->sendq
// attempt to fix hot i/o bug
            && ca->unack_attn_count < 10
        )
        {
            ca->unack_attn_count++;
            rc = device_attention(ca->dev, CSW_ATTN);
            if(ca->dev->ccwtrace)
                WRMSG(HHC01057, "D",
                    SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, rc);
        }
    }
    // end while(!sysblk.shutdown)

    LOG_THREAD_END( threadname );
    release_lock(&ca->lock);
    return NULL;
}
/*-------------------------------------------------------------------*/
/* Halt currently executing I/O command                              */
/*-------------------------------------------------------------------*/
static void commadpt_halt_or_clear( DEVBLK* dev )
{
    if (dev->busy)
    {
        // TODO: add code to halt/clear subchannel, if needed.
    }
}

/* The following 3 MSG functions ensure only 1 (one)  */
/* hardcoded instance exist for the same numbered msg */
/* that is issued on multiple situations              */
static void msg013e(DEVBLK *dev,char *kw,char *kv)
{
    WRMSG(HHC01007, "E", LCSS_DEVNUM,kw,kv);
}
/*-------------------------------------------------------------------*/
/* Device Initialization                                             */
/*-------------------------------------------------------------------*/
static int commadpt_init_handler (DEVBLK *dev, int argc, char *argv[])
{
    char thread_name[32];
    char thread_name2[32];
    int i;
    int rc;
    int pc; /* Parse code */
    int errcnt;
    union {
        int num;
        char text[MAX_PARSER_STRLEN+1];
    } res;

    /* For re-initialisation, close the existing file, if any */
    if (dev->fd >= 0)
        (dev->hnd->close)(dev);

    dev->devtype=0x3705;
    dev->excps = 0;
    if(dev->ccwtrace)
    {
        WRMSG(HHC01058,"D",
            LCSS_DEVNUM);
    }

    if(dev->commadpt!=NULL)
    {
        commadpt_clean_device(dev);
    }
    rc=commadpt_alloc_device(dev);
    if(rc<0)
    {
        WRMSG(HHC01011, "I", LCSS_DEVNUM);
        return(-1);
    }
    if(dev->ccwtrace)
    {
        WRMSG(HHC01059,"D",
            LCSS_DEVNUM);
    }
    errcnt=0;
    /*
     * Initialise ports & hosts
    */
    dev->commadpt->dlsw = 0;
    dev->commadpt->sfd=-1;
    dev->commadpt->lport=0;
    dev->commadpt->debug_sna=0;
    dev->commadpt->emu3791=0;
    dev->commadpt->locsuba = 0x3800;              /* local  subarea = 7 (maxsuba=31)        */
    dev->commadpt->rmtsuba = 0x4000;              /* remote subarea = 8 (maxsuba=31)        */
    strcpy(dev->commadpt->locncpnm,"MHP3705 ");   /* local  NCP name                        */
    strcpy(dev->commadpt->rmtncpnm,"MHPRMT1 ");   /* remote NCP name                        */
    prt_host_to_guest((BYTE*) dev->commadpt->locncpnm, (BYTE*) dev->commadpt->locncpnm, strlen(dev->commadpt->locncpnm)); /* convert to EBCDIC */
    prt_host_to_guest((BYTE*) dev->commadpt->rmtncpnm, (BYTE*) dev->commadpt->rmtncpnm, strlen(dev->commadpt->rmtncpnm)); /* convert to EBCDIC */
    dev->commadpt->idblk = 0x17;                  /* IDBLK of switched PU (default=0x017)   */
    dev->commadpt->idnum = 0x17;                  /* IDNUM of switched PU (default=0x00017) */
    dev->commadpt->unitsz = 256;                  /* I/O blocksize (must equal RRT KEYLN)   */
                                                  /* unitsz=256 is invalid for TCAM and     */
                                                  /* instable for VTAM. Default retained    */
                                                  /* for compatibility with previous        */
                                                  /* versions only.                         */
    dev->commadpt->ackspeed = 0;                  /* choose Max's original attn algorithm   */
    for(i=0;i<argc;i++)
    {
        pc=parser(ptab,argv[i],&res);
        if(pc<0)
        {
            WRMSG(HHC01012, "E",LCSS_DEVNUM,argv[i]);
            errcnt++;
            continue;
        }
        if(pc==0)
        {
            WRMSG(HHC01019, "E",LCSS_DEVNUM,argv[i]);
            errcnt++;
            continue;
        }
        switch(pc)
        {
            case COMMADPT_KW_DEBUG:
                if (res.text[0] == 'y' || res.text[0] == 'Y')
                    dev->commadpt->debug_sna = 1;
                else
                    dev->commadpt->debug_sna = 0;
                break;
            case COMMADPT_KW_LPORT:
                rc=commadpt_getport(res.text);
                if(rc<0)
                {
                    errcnt++;
                    msg013e(dev,"LPORT",res.text);
                    break;
                }
                dev->commadpt->lport=rc;
                break;
            case COMMADPT_KW_LHOST:
                if(strcmp(res.text,"*")==0)
                {
                    dev->commadpt->lhost=INADDR_ANY;
                    break;
                }
                rc=commadpt_getaddr(&dev->commadpt->lhost,res.text);
                if(rc!=0)
                {
                    msg013e(dev,"LHOST",res.text);
                    errcnt++;
                }
                break;
            case COMMADPT_KW_EMU3791:
                if(strcasecmp(res.text,"yes")==0 || strcmp(res.text,"1"))
                    dev->commadpt->emu3791=1;
                break;
            case COMMADPT_KW_LOCSUBA:
                    dev->commadpt->locsuba = (atoi(res.text)<<11); /* (maxsuba=31) */
                break;
            case COMMADPT_KW_RMTSUBA:
                    dev->commadpt->rmtsuba = (atoi(res.text)<<11); /* (maxsuba=31) */
                break;
            case COMMADPT_KW_LOCNCPNM:
                    strcpy(dev->commadpt->locncpnm,"        ");
                    strcpy(dev->commadpt->locncpnm,res.text);
                    memcpy(&dev->commadpt->locncpnm[strlen(res.text)]," ",1);
                    prt_host_to_guest((BYTE*) dev->commadpt->locncpnm, (BYTE*) dev->commadpt->locncpnm, strlen(dev->commadpt->locncpnm));
                break;
            case COMMADPT_KW_RMTNCPNM:
                    strcpy(dev->commadpt->rmtncpnm,"        ");
                    strcpy(dev->commadpt->rmtncpnm,res.text);
                    memcpy(&dev->commadpt->rmtncpnm[strlen(res.text)]," ",1);
                    prt_host_to_guest((BYTE*) dev->commadpt->rmtncpnm, (BYTE*) dev->commadpt->rmtncpnm, strlen(dev->commadpt->rmtncpnm));
                break;
            case COMMADPT_KW_IDBLK:
                    sscanf(res.text,"%3x",&dev->commadpt->idblk);
                break;
            case COMMADPT_KW_IDNUM:
                    sscanf(res.text,"%5x",&dev->commadpt->idnum);
                break;
            case COMMADPT_KW_UNITSZ:
                    dev->commadpt->unitsz = atoi(res.text);
                break;
            case COMMADPT_KW_ACKSPEED:
                    dev->commadpt->ackspeed = atoi(res.text);
                break;
            case COMMADPT_KW_DLSW:
                if (res.text[0] == 'y' || res.text[0] == 'Y')
                    dev->commadpt->dlsw = 1;
                break;
            default:
                break;
        }
    }
    if(errcnt>0)
    {
        WRMSG(HHC01014, "I",LCSS_DEVNUM);
        return -1;
    }
    dev->bufsize=dev->commadpt->unitsz;
    dev->numsense=2;
    memset(dev->sense,0,sizeof(dev->sense));

    init_bufpool(dev->commadpt);

    dev->commadpt->devnum=dev->devnum;

    /* Initialize the CA lock */
    initialize_lock(&dev->commadpt->lock);

    /* Initialise thread->I/O & halt initiation EVB */
    initialize_condition(&dev->commadpt->ipc);
    initialize_condition(&dev->commadpt->ipc_halt);

    /* Allocate I/O -> Thread signaling pipe */
    VERIFY(!create_pipe(dev->commadpt->pipe));

    /* Obtain the CA lock */
    obtain_lock(&dev->commadpt->lock);

    if (dev->commadpt->dlsw)
    {
        /* Start the DLSw worker thread */

        dev->commadpt->lport = DLSW_PORT;
        dev->commadpt->rport = DLSW_PORT;

        /* Set thread-name for debugging purposes */
        snprintf(thread_name,sizeof(thread_name),
            "commadpt %1d:%04X dlsw_thread",dev->ssid,dev->devnum);
        thread_name[sizeof(thread_name)-1]=0;

        rc = create_thread(&dev->commadpt->tthread,&sysblk.detattr,dlsw_thread,dev->commadpt,thread_name);
        if(rc)
        {
            WRMSG(HHC00102, "E" ,strerror(rc));
            release_lock(&dev->commadpt->lock);
            return -1;
        }
    }
    else
    {
        /* Start the telnet worker thread */

        /* Set thread-name for debugging purposes */
        snprintf(thread_name2,sizeof(thread_name2),
            "commadpt %1d:%04X thread2",dev->ssid,dev->devnum);
        thread_name2[sizeof(thread_name2)-1]=0;

        rc = create_thread(&dev->commadpt->tthread,&sysblk.detattr,telnet_thread,dev->commadpt,thread_name2);
        if(rc)
        {
            WRMSG(HHC00102, "E" ,strerror(rc));
            release_lock(&dev->commadpt->lock);
            return -1;
        }
    }

    /* Start the async worker thread */

    /* Set thread-name for debugging purposes */
    snprintf(thread_name,sizeof(thread_name),
        "commadpt %1d:%04X thread",dev->ssid,dev->devnum);
    thread_name[sizeof(thread_name)-1]=0;

    rc = create_thread(&dev->commadpt->cthread,&sysblk.detattr,commadpt_thread,dev->commadpt,thread_name);
    if(rc)
    {
        WRMSG(HHC00102, "E", strerror(rc));
        release_lock(&dev->commadpt->lock);
        return -1;
    }
    dev->commadpt->have_cthread=1;

    /* Release the CA lock */
    release_lock(&dev->commadpt->lock);
    /* Indicate succesfull completion */
    return 0;
}

/*-------------------------------------------------------------------*/
/* Query the device definition                                       */
/*-------------------------------------------------------------------*/
static void commadpt_query_device (DEVBLK *dev, char **class,
                int buflen, char *buffer)
{
    char filename[ PATH_MAX + 1 ];      /* full path or just name    */

    BEGIN_DEVICE_CLASS_QUERY( "LINE", dev, class, buflen, buffer );

    snprintf(buffer,buflen,"Read count=%d, Write count=%d IO[%"PRIu64"]",
        dev->commadpt->read_ccw_count, dev->commadpt->write_ccw_count, dev->excps );
}

/*-------------------------------------------------------------------*/
/* Close the device                                                  */
/* Invoked by HERCULES shutdown & DEVINIT processing                 */
/*-------------------------------------------------------------------*/
static int commadpt_close_device( DEVBLK* dev )
{
    if (dev->ccwtrace)
    {
        // "%1d:%04X COMM: closing down"
        WRMSG( HHC01060, "D", LCSS_DEVNUM );
    }

    obtain_lock( &dev->commadpt->lock );
    {
        /* Terminate current I/O thread if necessary */
        if (dev->busy)
            commadpt_halt_or_clear( dev );

        free_bufpool( dev->commadpt );
    }
    release_lock( &dev->commadpt->lock );

    /* Free all work storage */
    commadpt_clean_device( dev );

    /* Indicate to hercules the device is no longer opened */
    dev->fd = -1;

    if (dev->ccwtrace)
    {
        // "%1d:%04X COMM: closed down"
        WRMSG( HHC01061, "D", LCSS_DEVNUM );
    }
    return 0;
}

void make_seq (COMMADPT * ca, BYTE * reqptr) {
    if (reqptr[4] == (ca->locsuba >> 8)) { /* local NCP  */
        reqptr[6] = (unsigned char)(++ca->ncpa_sscp_seqn >> 8) & 0xff;
        reqptr[7] = (unsigned char)(  ca->ncpa_sscp_seqn     ) & 0xff;
    } else
    if (reqptr[4] == (ca->rmtsuba >> 8)){ /* remote NCP */
        reqptr[6] = (unsigned char)(++ca->ncpb_sscp_seqn >> 8) & 0xff;
        reqptr[7] = (unsigned char)(  ca->ncpb_sscp_seqn     ) & 0xff;
    }
}

static void format_sna (BYTE * requestp, char * tag, U16 ssid, U16 devnum) {
    char     fmtbuf[32];
    char     fmtbuf2[32];
    char     fmtbuf3[32];
    char     fmtbuf4[32];
//  char     fmtbuf5[256];
    char     fmtbuf6[32];
    char     *ru_type="";
    int      len;
    sprintf(fmtbuf, "%02X%02X %02X%02X %02X%02X %02X%02X %02X%02X",
        requestp[0], requestp[1], requestp[2], requestp[3], requestp[4], requestp[5], requestp[6], requestp[7], requestp[8], requestp[9]);
    sprintf(fmtbuf2, "%02X%02X%02X",
        requestp[10], requestp[11], requestp[12]);
    len = (requestp[8] << 8) + requestp[9];
    len -= 3;   /* for len of ru only */
    sprintf(fmtbuf3, "%02X", requestp[13]);
    sprintf(fmtbuf4, "%02X", requestp[14]);
    if (len > 1)
        STRLCAT( fmtbuf3, fmtbuf4 );
    sprintf(fmtbuf4, "%02X", requestp[15]);
    if (len > 2)
        STRLCAT( fmtbuf3, fmtbuf4 );

    if (requestp[13] == 0x04)
        ru_type = "LUSTAT";
    if (requestp[13] == 0x05)    // RTR?
        ru_type = "LSA";
    if (requestp[13] == 0x07)
        ru_type = "ANSC";
    if (requestp[13] == 0x0D)
        ru_type = "ACTLU";
    if (requestp[13] == 0x0E)
        ru_type = "DACTLU";
    if (requestp[13] == 0x11)
        ru_type = "ACTPU";
    if (requestp[13] == 0x12)
        ru_type = "DACTPU";
    if (requestp[13] == 0x14)
        ru_type = "ACTCDRM";
    if (requestp[13] == 0x15)
        ru_type = "DACTCDRM";
    if (requestp[13] == 0x31)
        ru_type = "BIND";
    if (requestp[13] == 0x32)
        ru_type = "UNBIND";
    if (requestp[13] == 0x70)
        ru_type = "BIS";
    if (requestp[13] == 0x80)
        ru_type = "QEC";
    if (requestp[13] == 0x81)
        ru_type = "QC";
    if (requestp[13] == 0x82)
        ru_type = "RELQ";
    if (requestp[13] == 0x83)
        ru_type = "CANCEL";
    if (requestp[13] == 0x84)
        ru_type = "CHASE";
    if (requestp[13] == 0xA0)
        ru_type = "SDT";
    if (requestp[13] == 0xA1)
        ru_type = "CLEAR";
    if (requestp[13] == 0xA3)
        ru_type = "RQR";
    if (requestp[13] == 0xC0)
        ru_type = "CRV";
    if (requestp[13] == 0xC2)
        ru_type = "RSHUTD";
    if (requestp[13] == 0xC8)
        ru_type = "BID";

    if (!memcmp(&requestp[13], R010201, 3))
        ru_type = "CONTACT";
    if (!memcmp(&requestp[13], R010202, 3))
        ru_type = "DISCONTACT";
    if (!memcmp(&requestp[13], R010203, 3))
        ru_type = "IPLINIT";
    if (!memcmp(&requestp[13], R010204, 3))
        ru_type = "IPLTEXT";
    if (!memcmp(&requestp[13], R010205, 3))
        ru_type = "IPLFINAL";
    if (!memcmp(&requestp[13], R010206, 3))
        ru_type = "DUMPINIT";
    if (!memcmp(&requestp[13], R010207, 3))
        ru_type = "DUMPTEXT";
    if (!memcmp(&requestp[13], R010208, 3))
        ru_type = "DUMPFINAL";
    if (!memcmp(&requestp[13], R010209, 3))
        ru_type = "RPO";
    if (!memcmp(&requestp[13], R01020A, 3))
        ru_type = "ACTLINK";
    if (!memcmp(&requestp[13], R01020B, 3))
        ru_type = "DACTLINK";
    if (!memcmp(&requestp[13], R01020C, 3))
        ru_type = "CESLOW";
    if (!memcmp(&requestp[13], R01020D, 3))
        ru_type = "CEXSLOW";
    if (!memcmp(&requestp[13], R01020E, 3))
        ru_type = "CONNOUT";
    if (!memcmp(&requestp[13], R01020F, 3))
        ru_type = "ABCONN";
    if (!memcmp(&requestp[13], R010211, 3)) {
        sprintf(fmtbuf6, "%s[%02x]", "SETCV", requestp[18]);
        ru_type = fmtbuf6;
        if ((requestp[10] & 0x80) != 0)
            ru_type = "SETCV";
    }
    if (!memcmp(&requestp[13], R010214, 3))
        ru_type = "ESLOW";
    if (!memcmp(&requestp[13], R010215, 3))
        ru_type = "EXSLOW";
    if (!memcmp(&requestp[13], R010216, 3))
        ru_type = "ACTCONNIN";
    if (!memcmp(&requestp[13], R010217, 3))
        ru_type = "DACTCONNIN";
    if (!memcmp(&requestp[13], R010218, 3))
        ru_type = "ABCONNOUT";
    if (!memcmp(&requestp[13], R010219, 3))
        ru_type = "ANA";
    if (!memcmp(&requestp[13], R01021A, 3))
        ru_type = "FNA";
    if (!memcmp(&requestp[13], R01021B, 3))
        ru_type = "REQDISCONT";
    if (!memcmp(&requestp[13], R010280, 3))
        ru_type = "CONTACTED";
    if (!memcmp(&requestp[13], R010281, 3))
        ru_type = "INOP";
    if (!memcmp(&requestp[13], R010284, 3))
        ru_type = "REQCONT";

    if (!memcmp(&requestp[13], R010301, 3))
        ru_type = "EXECTEST";
    if (!memcmp(&requestp[13], R010302, 3))
        ru_type = "ACTTRACE";
    if (!memcmp(&requestp[13], R010303, 3))
        ru_type = "DACTTRACE";
    if (!memcmp(&requestp[13], R010331, 3))
        ru_type = "DISPSTOR";
    if (!memcmp(&requestp[13], R010334, 3))
        ru_type = "RECSTOR";
    if (!memcmp(&requestp[13], R010380, 3))
        ru_type = "REQTEST";
    if (!memcmp(&requestp[13], R010381, 3))
        ru_type = "RECMS";
    if (!memcmp(&requestp[13], R010382, 3))
        ru_type = "RECTD";
    if (!memcmp(&requestp[13], R010383, 3))
        ru_type = "RECTRD";

    if (!memcmp(&requestp[13], R010480, 3))
        ru_type = "RECMD";
    if (!memcmp(&requestp[13], R010604, 3))
        ru_type = "NSPE";
    if (!memcmp(&requestp[13], R010681, 3))
        ru_type = "INIT-SELF";

    if (!memcmp(&requestp[13], R410210, 3))
        ru_type = "RNAA";
    if (!memcmp(&requestp[13], R410222, 3))
        ru_type = "ISETCV";
    if (!memcmp(&requestp[13], R410285, 3))
        ru_type = "NSLSA";

    if (!memcmp(&requestp[13], R410304, 3))
        ru_type = "REQMS";
    if (!memcmp(&requestp[13], R410384, 3))
        ru_type = "RECFMS";

    if (!memcmp(&requestp[13], R810601, 3))
        ru_type = "CINIT";
    if (!memcmp(&requestp[13], R810602, 3))
        ru_type = "CTERM";
    if (!memcmp(&requestp[13], R810620, 3))
        ru_type = "NOTIFY";
    if (!memcmp(&requestp[13], R810629, 3))
        ru_type = "CLEANUP";
    if (!memcmp(&requestp[13], R810680, 3))
        ru_type = "INIT-OTHER";
    if (!memcmp(&requestp[13], R810681, 3))
        ru_type = "INIT-SELF";
    if (!memcmp(&requestp[13], R810685, 3))
        ru_type = "BINDF";

    if (!memcmp(&requestp[13], R818620, 3))
        ru_type = "NOTIFY";
    if (!memcmp(&requestp[13], R818627, 3))
        ru_type = "DSRLST";
    if (!memcmp(&requestp[13], R818640, 3))
        ru_type = "INIT-OTHER-CD";
    if (!memcmp(&requestp[13], R818641, 3))
        ru_type = "CDINIT";
    if (!memcmp(&requestp[13], R818643, 3))
        ru_type = "CDTERM";
    if (!memcmp(&requestp[13], R818645, 3))
        ru_type = "CDSESSSF";
    if (!memcmp(&requestp[13], R818646, 3))
        ru_type = "CDSESSST";
    if (!memcmp(&requestp[13], R818647, 3))
        ru_type = "CDSESSSTF";
    if (!memcmp(&requestp[13], R818648, 3))
        ru_type = "CDSESSEND";
    if (!memcmp(&requestp[13], R818649, 3))
        ru_type = "CDTAKED";
    if (!memcmp(&requestp[13], R81864A, 3))
        ru_type = "CDTAKEDC";
    if (!memcmp(&requestp[13], R81864B, 3))
        ru_type = "CDCINIT";

    if ((requestp[10] & 0x08) == 0)
        ru_type = "";
    WRMSG(HHC01062,"D",
        SSID_TO_LCSS(ssid), devnum, tag, fmtbuf, fmtbuf2, fmtbuf3, ru_type);
}

static void make_sna_requests2 (COMMADPT *ca) {
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;
    int     bufp = 0;
    while (ca->inpbufl > 0) {
        eleptr = get_bufpool(&ca->freeq);
        if (!eleptr)  {
            WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA request2");
            return;
        }
        respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

        /* first do the ten-byte FID1 TH */
        respbuf[0] = 0x1C;
        respbuf[1] = 0x00;
        respbuf[2] = ca->tso_addr0;   // daf
        respbuf[3] = ca->tso_addr1;
        respbuf[4] = ca->lu_addr0;   // oaf
        respbuf[5] = ca->lu_addr1;   // oaf
        respbuf[6] = (unsigned char)(++ca->lu_lu_seqn >> 8) & 0xff;
        respbuf[7] = (unsigned char)(  ca->lu_lu_seqn     ) & 0xff;

        /* do RH */
        respbuf[10] = 0x00;
        if (!bufp) {
            respbuf[10] |= 0x02;      /* set first in chain */
        }
        respbuf[11] = 0x90;
        respbuf[12] = 0x00;

        /* do RU */

        // FIXME - max. ru_size should be based on BIND settings
        // A true fix would also require code changes to READ CCW processing
        // including possibly (gasp) segmenting long PIUs into multiple BTUs
        // JW: still not fixed but unitsz is now an external parameter
        //     to allow easier modification
        ru_size = min(ca->unitsz-(BUFPD+10+3),ca->inpbufl);
        ru_ptr = &respbuf[13];

        if (!ca->bindflag) {
            // send as character-coded logon to SSCP
            if (ru_size > 0 && (ca->inpbuf[ca->inpbufl-1] == 0x0d || ca->inpbuf[ca->inpbufl-1] == 0x25)) {
                ru_size--;
            }
            if (ru_size > 0 && (ca->inpbuf[ca->inpbufl-1] == 0x0d || ca->inpbuf[ca->inpbufl-1] == 0x25)) {
                ru_size--;
            }
            respbuf[2] = ca->sscp_addr0;
            respbuf[3] = ca->sscp_addr1;
            respbuf[11] = 0x80;
            respbuf[12] = 0x00;
        }
        memcpy(ru_ptr, &ca->inpbuf[bufp], ru_size);
        bufp        += ru_size;
        ca->inpbufl -= ru_size;
        if (!ca->is_3270) {
            ca->inpbufl = 0;
        }
        if (!ca->inpbufl) {
            respbuf[10] |= 0x01;      /* set last in chain */
            if (ca->bindflag) {
                respbuf[12] |= 0x20;      /* set CD */
            }
        }

        /* set length field in TH */
        ru_size += 3;   /* for RH */
        respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
        respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

        put_bufpool(&ca->sendq, eleptr);
    } /* end of while (ca->inpbufl > 0) */
}

static void sna_sig (COMMADPT *ca) {
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;
    if (!ca->telnet_int) return;
    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
        WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA SIG");
        return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    /* first do the ten-byte FID1 TH */
    respbuf[0] = 0x1D;
    respbuf[1] = 0x00;
    respbuf[2] = ca->tso_addr0;   // daf
    respbuf[3] = ca->tso_addr1;
    respbuf[4] = ca->lu_addr0;   // oaf
    respbuf[5] = ca->lu_addr1;   // oaf
    respbuf[6] = 0x11;
    respbuf[7] = 0x11;

    /* do RH */
    respbuf[10] = 0x4B;
    respbuf[11] = 0x80;
    respbuf[12] = 0x00;

    /* do RU */
    ru_size = 0;
    ru_ptr = &respbuf[13];

    ru_ptr[ru_size++] = 0xc9;      // SIG
    ru_ptr[ru_size++] = 0x00;
    ru_ptr[ru_size++] = 0x01;

    ru_size += 3;   /* for RH */
    respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
    respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

    put_bufpool(&ca->sendq, eleptr);
    ca->telnet_int = 0;
}

static void sna_reqcont (COMMADPT *ca, BYTE pu_type, U32 stids) {
    /* send type flag: 0=REQCONT 1=REQDISCONT */
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;
    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
        WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA REQCONT");
        return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    /* first do the ten-byte FID1 TH */
    respbuf[0] = 0x1C;
    respbuf[1] = 0x00;
    respbuf[2] = ca->sscp_addr0;   // daf
    respbuf[3] = ca->sscp_addr1;
    respbuf[4] = ca->ncp_addr0;    // oaf
    respbuf[5] = ca->ncp_addr1;
    make_seq(ca, respbuf);

    /* do RH */
    respbuf[10] = 0x0b;
    respbuf[11] = 0x00;
    respbuf[12] = 0x00;

    /* do RU */
    ru_size = 0;
    ru_ptr = &respbuf[13];
    ru_ptr[ru_size++] = 0x01;      // REQCONT (REQUEST CONTACT)
    ru_ptr[ru_size++] = 0x02;
    ru_ptr[ru_size++] = 0x84;

    ru_ptr[ru_size++] = (ca->rmtsuba >> 8); // network address of link
    ru_ptr[ru_size++] = 0x01;

    ru_ptr[ru_size++] = pu_type;      // PU type

    ru_ptr[ru_size++] = 0x00;

    ru_ptr[ru_size++] = (stids >> 24) &0xff;
    ru_ptr[ru_size++] = (stids >> 16) &0xff;
    ru_ptr[ru_size++] = (stids >>  8) &0xff;
    ru_ptr[ru_size++] =  stids        &0xff;
    ru_size += 3;   /* for RH */
    respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
    respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

    put_bufpool(&ca->sendq, eleptr);
    ca->telnet_int = 0;
}

static void sna_inop (COMMADPT *ca) {
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;
    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
        WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA INOP");
        return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    /* first do the ten-byte FID1 TH */
    respbuf[0] = 0x1C;
    respbuf[1] = 0x00;
    respbuf[2] = ca->sscp_addr0;   // daf
    respbuf[3] = ca->sscp_addr1;
    respbuf[4] = ca->ncp_addr0;    // oaf
    respbuf[5] = ca->ncp_addr1;
    // set seq no.
    make_seq(ca, respbuf);
    /* do RH */
    respbuf[10] = 0x0B;
    respbuf[11] = 0x00;
    respbuf[12] = 0x00;

    /* do RU */
    ru_size = 0;
    ru_ptr = &respbuf[13];

    ru_ptr[ru_size++] = 0x01;      // INOP
    ru_ptr[ru_size++] = 0x02;
    ru_ptr[ru_size++] = 0x81;
    ru_ptr[ru_size++] = ca->pu_addr0;
    ru_ptr[ru_size++] = ca->pu_addr1;
    ru_ptr[ru_size++] = 0x01;      // format/reason

    ru_size += 3;   /* for RH */
    respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
    respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

    put_bufpool(&ca->sendq, eleptr);
}

void make_sna_requests (BYTE * requestp, COMMADPT *ca) {
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;
    int     i;
    if (memcmp(&requestp[13], R010201, 3)) return;   // we only want to process CONTACT
    if (ca->dlsw)
    {
        for (i = 0; i < 20; i++)
        {
            DLSW_DEBUG("CONTACT %02X%02X, map[%d] = %02X%02X\n", requestp[16], requestp[17], i, ca->dlsw_map[i].addr0, ca->dlsw_map[i].addr1);
            if ((ca->dlsw_map[i].valid) && (ca->dlsw_map[i].addr0 == requestp[16]) && (ca->dlsw_map[i].addr1 == requestp[17]))
            {
                dlsw_contact(ca, requestp);
                return;
            }
        }
    }
    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
            WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA request");
            return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    /* first do the ten-byte FID1 TH */
//    respbuf[0] = requestp[0];
//    respbuf[1] = requestp[1];
    respbuf[0] = 0x1c;
    respbuf[1] = 0x00;
    respbuf[2] = requestp[4];   // daf
    respbuf[3] = requestp[5];
    respbuf[4] = requestp[2];   // oaf
    respbuf[5] = requestp[3];
    make_seq(ca, respbuf);
    /* do RH */
    respbuf[10] = requestp[10];
    respbuf[11] = requestp[11];
    respbuf[11] = 0x00;
    respbuf[12] = requestp[12];

    /* make a CONTACTED RU */
    ru_size = 0;
    ru_ptr = &respbuf[13];
    ru_ptr[ru_size++] = 0x01;
    ru_ptr[ru_size++] = 0x02;
    ru_ptr[ru_size++] = 0x80;
    ru_ptr[ru_size++] = requestp[16];
    ru_ptr[ru_size++] = requestp[17];
    ru_ptr[ru_size++] = 0x01;

    /* set length field in TH */
    ru_size += 3;   /* for RH */
    respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
    respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

    put_bufpool(&ca->sendq, eleptr);
}

void make_sna_response (BYTE * requestp, COMMADPT *ca) {
    BYTE    *respbuf;
    BYTE    *ru_ptr;
    int     ru_size;
    void    *eleptr;
    BYTE    obuf[4096];
    BYTE    buf[BUFLEN_3270];
    int     amt;
    int     i1;
    int     i;

    if (ca->dlsw) {
        for (i = 0; i < 20; i++) {
            if ((ca->dlsw_map[i].valid) && (ca->dlsw_map[i].addr0 == requestp[2]) && (ca->dlsw_map[i].addr1 == requestp[3])) {
                if (ca->circuit) {
                    dlsw_inforeq(ca, requestp, ca->dlsw_map[i].laddr);
                    return;
                }
            }
        }
        logmsg ("WARNING: DLSw packet lost, requestp[2] = 0x%x, requestp[3] = 0x%x\n", requestp[2], requestp[3]);
    }

    if ((requestp[10] & 0x80) != 0) return;   // disregard if this is a resp.
    if ((requestp[10] & (unsigned char)0xfc) == 0x00 && requestp[2] == ca->lu_addr0 && requestp[3] == ca->lu_addr1 && ca->sfd > 0) {   /* if type=data, and DAF matches up, and socket exists */
        amt = (requestp[8] << 8) + requestp[9];
        amt -= 3;
        if (ca->is_3270) {
            memcpy(buf, &requestp[13], amt);
            /* Double up any IAC bytes in the data */
            amt = double_up_iac (buf, amt);
            /* Append telnet EOR marker at end of data */
            if ((requestp[10] & 0x01) == 0x01) {   /* if last-in-chain is set */
                buf[amt++] = IAC;
                buf[amt++] = EOR_MARK;
            }
            /* Send the data to the client */
            logdump ("SEND", ca->dev, buf, amt);
            write_socket(ca->sfd,buf,amt);
        } else {
            // convert data portion to ASCII and write to remote user
            if (amt > 0) {
                memcpy(obuf, &requestp[13], amt);
                for (i1=0; i1<amt; i1++) {
                    obuf[i1] = guest_to_host(obuf[i1]);
                }
                logdump ("SEND", ca->dev, obuf, amt);
                write_socket(ca->sfd,obuf,amt);
            }
        }
    }
    if ((requestp[11] & 0xf0) != 0x80) return;   // disregard if not DR1 requested

    eleptr = get_bufpool(&ca->freeq);
    if (!eleptr)  {
        WRMSG(HHC01020, "E", SSID_TO_LCSS(ca->dev->ssid), ca->dev->devnum, "SNA response");
        return;
    }
    respbuf = SIZEOF_INT_P + (BYTE*)eleptr;

    /* first do the ten-byte FID1 TH */
    respbuf[0] = requestp[0];
    respbuf[1] = requestp[1];
    respbuf[2] = requestp[4];   // daf
    respbuf[3] = requestp[5];
    respbuf[4] = requestp[2];   // oaf
    respbuf[5] = requestp[3];
    respbuf[6] = requestp[6];   // seq #
    respbuf[7] = requestp[7];

    /* do RH */
    respbuf[10] = requestp[10];
    respbuf[10] |= 0x83;         // indicate this is a resp.
    respbuf[11] = requestp[11];
//    respbuf[12] = requestp[12];
    respbuf[12] = 0x00;

    /* do RU */
    ru_size = 0;
    ru_ptr = &respbuf[13];
    if ((requestp[10] & 0x08) != 0)
        ru_ptr[ru_size++] = requestp[13];
    if (requestp[13] == 0x11 && requestp[14] == 0x02) {   /* ACTPU (NCP)*/
        ca->ncp_addr0 = requestp[2];
        ca->ncp_addr1 = requestp[3];
//        ca->ncp_sscp_seqn = 0;
        ru_ptr[ru_size++] = 0x02;
        if (requestp[2] == (ca->rmtsuba >> 8)){      /* remote NCP    */
            memcpy(&ru_ptr[ru_size],ca->rmtncpnm,8); /* load mod name */
            ru_size += 8;
            ca->ncpb_sscp_seqn = 0;
        } else
        if (requestp[2] == (ca->locsuba >> 8)){      /* local  NCP    */
            memcpy(&ru_ptr[ru_size],ca->locncpnm,8); /* load mod name */
            ru_size += 8;
            ca->ncpa_sscp_seqn = 0;
        }
    }
    if (requestp[13] == 0x11 && requestp[14] == 0x01) {   /* ACTPU (PU)*/
        ru_ptr[ru_size++] = 0x01;
        /* save daf as our own net addr */
        ca->pu_addr0 = requestp[2];
        ca->pu_addr1 = requestp[3];
    }
    if (requestp[13] == 0x01) {   /* 01XXXX Network Services */
        ru_ptr[ru_size++] = requestp[14];
        ru_ptr[ru_size++] = requestp[15];
    }
    if (!memcmp(&requestp[13], R010219, 3) && ca->sfd > 0) {   /* ANA */
//        if (!ca->is_3270) {
//            connect_message(ca->sfd, (requestp[20] << 8) + requestp[21], 0);
//        }
    }
    if (!memcmp(&requestp[13], R010211, 3)) {   /* SETCV */
        if (requestp[18] == 0x3) {   /* secondary station control vector */
            DLSW_DEBUG("Processing SETCV (secondary station)\n");
            for (i = 0; i < 20; i++) {
                if (ca->dlsw_map[i].valid == 0) {
                    ca->dlsw_map[i].addr0 = requestp[16];
                    ca->dlsw_map[i].addr1 = requestp[17];
                    ca->pu_addr0 = requestp[16];
                    ca->pu_addr1 = requestp[17];
                    ca->dlsw_map[i].laddr = 0;
                    ca->dlsw_map[i].valid = 1;
                    DLSW_DEBUG("Added mapping at index %d from %02X%02X to %02X\n",
                            i, requestp[16], requestp[17], 0);
                    break;
                }
            }
        }
        if (requestp[18] == 0x4) {   /* LU control vector */
            DLSW_DEBUG("Processing SETCV (LU)\n");
            for (i = 0; i < 20; i++) {
                if (ca->dlsw_map[i].valid == 0) {
                    ca->dlsw_map[i].addr0 = requestp[16];
                    ca->dlsw_map[i].addr1 = requestp[17];
                    ca->dlsw_map[i].laddr = requestp[19];
                    ca->dlsw_map[i].valid = 1;
                    DLSW_DEBUG("Added mapping at index %d from %02X%02X to %02X\n",
                            i, requestp[16], requestp[17], requestp[19]);
                    break;
                }
            }
        }
    }
    if (requestp[13] == 0x0D) {   /* ACTLU */
        /* save daf as our own net addr */
        ca->lu_addr0 = requestp[2];
        ca->lu_addr1 = requestp[3];
        /* save oaf as our sscp net addr */
        ca->sscp_addr0 = requestp[4];
        ca->sscp_addr1 = requestp[5];

        ca->lu_sscp_seqn = 0;
        ca->bindflag = 0;
    }
    if (requestp[13] == 0x0E || !memcmp(&requestp[13], R01020F, 3)) {  // DACTLU or ABCONN
//        if (!ca->is_3270) {
//            connect_message(ca->sfd, 0, 1);
//        }
        ca->hangup = 1;
    }
    if (requestp[13] == 0x31) {   /* BIND */
        /* save oaf from BIND request */
        ca->tso_addr0 = requestp[4];
        ca->tso_addr1 = requestp[5];
        ca->lu_lu_seqn = 0;
        ca->bindflag = 1;
    }
    if (requestp[13] == 0x32 && requestp[14] != 0x02) {   /* BIND */
        ca->bindflag = 0;
    }

    /* set length field in TH */
    ru_size += 3;   /* for RH */
    respbuf[8] = (unsigned char)(ru_size >> 8) & 0xff;
    respbuf[9] = (unsigned char)(ru_size     ) & 0xff;

    put_bufpool(&ca->sendq, eleptr);
}

static void th_remap(enum fid_remap r, BYTE * thptr, U16 locsuba)
{ /* for 3791 support, remaps SNA FID1 <--> FID2 TH headers */
int     thmpf;
int     thm2;
int     thdaf;
int     thoaf;
int     thsnf;
int     len;

    if (r == MAP_FID1_FID2)
    {
        thmpf = thptr[0];
        thm2  = thptr[1];
        thdaf = (thptr[2] << 8) + thptr[3];
        thoaf = (thptr[4] << 8) + thptr[5];
        thsnf = (thptr[6] << 8) + thptr[7];
        len = (thptr[8] << 8) + thptr[9];
        len += 10;
        thptr[0] = (len >> 8) & 0xff;
        thptr[1] = len & 0xff;
        thptr[2] = 0x00;
        thptr[3] = 0x00;
        thptr[4] = 0x20 | (thmpf & 0x0f);
        thptr[5] = thm2;
        thptr[6] = thdaf & 0xff;
        thptr[7] = thoaf & 0xff;
        thptr[8] = (thsnf >> 8) & 0xff;
        thptr[9] = thsnf & 0xff;
    }
    else
    { /* map fid2 to fid1 */
        len = (thptr[0] << 8) + thptr[1];
        thmpf = thptr[4];
        thm2  = thptr[5];
        thdaf = thptr[6];
        thoaf = thptr[7];
        thsnf = (thptr[8] << 8) + thptr[9];
        thdaf |= locsuba;
        thoaf |= 0x0800;   /* SSCP subarea = 1 (maxsuba=31) */
        len -= 10;
        thptr[0] = 0x10 | (thmpf & 0x0f);
        thptr[1] = thm2;
        thptr[2] = (thdaf >> 8) & 0xff;
        thptr[3] = thdaf & 0xff;
        thptr[4] = (thoaf >> 8) & 0xff;
        thptr[5] = thoaf & 0xff;
        thptr[6] = (thsnf >> 8) & 0xff;
        thptr[7] = thsnf & 0xff;
        thptr[8] = (len >> 8) & 0xff;
        thptr[9] = len & 0xff;
    }
}

/*-------------------------------------------------------------------*/
/* Execute a Channel Command Word                                    */
/*-------------------------------------------------------------------*/
static void commadpt_execute_ccw (DEVBLK *dev, BYTE code, BYTE flags,
        BYTE chained, U32 count, BYTE prevcode, int ccwseq,
        BYTE *iobuf, BYTE *more, BYTE *unitstat, U32 *residual)
{
U32 num;                        /* Work : Actual CCW transfer count                   */
BYTE    *piudata;
int     piusize;
void    *eleptr;
int     llsize;

    UNREFERENCED(flags);
    UNREFERENCED(chained);
    UNREFERENCED(prevcode);
    UNREFERENCED(ccwseq);

    *residual = 0;

    /*
     * Obtain the COMMADPT lock
     */
    if(dev->ccwtrace)
    {
        WRMSG(HHC01063,"D",
            LCSS_DEVNUM,code);
    }
    obtain_lock(&dev->commadpt->lock);
    switch (code) {
        /*---------------------------------------------------------------*/
        /* BASIC SENSE                                                   */
        /*---------------------------------------------------------------*/
        case 0x04:
            dev->commadpt->unack_attn_count = 0;
            num=count<dev->numsense?count:dev->numsense;
            *more=count<dev->numsense?1:0;
            memcpy(iobuf,dev->sense,num);
            *residual=count-num;
            *unitstat=CSW_CE|CSW_DE;
            break;

        /*---------------------------------------------------------------*/
        /* READ type CCWs                                                */
        /*---------------------------------------------------------------*/
        case 0x02:   /* READ */
            dev->commadpt->read_ccw_count++;
            dev->commadpt->unack_attn_count = 0;
            *more = 0;
            make_sna_requests2(dev->commadpt);
            sna_sig(dev->commadpt);
            eleptr = get_bufpool(&dev->commadpt->sendq);
            *residual=count;
            if (eleptr) {
                piudata = SIZEOF_INT_P + (BYTE*)eleptr;
                piusize = (piudata[8] << 8) + piudata[9];
                piusize += 10;    // for FID1 TH
                iobuf[0] = BUFPD;
                memcpy (&iobuf[BUFPD], piudata, piusize);
                if (dev->commadpt->emu3791) {
                    llsize = piusize + BUFPD;
                    iobuf[0] = (llsize >> 8) & 0xff;
                    iobuf[1] = llsize & 0xff;
                    th_remap(MAP_FID1_FID2, &iobuf[BUFPD], dev->commadpt->locsuba);
                }
                *residual=count - (piusize + BUFPD);
                logdump("READ", dev, &iobuf[BUFPD], piusize);
                if (dev->commadpt->debug_sna)
                    format_sna(piudata, "RD", dev->ssid, dev->devnum);
                put_bufpool(&dev->commadpt->freeq, eleptr);
            }
            *unitstat  = CSW_CE | CSW_DE | CSW_UX;
            break;

        /*---------------------------------------------------------------*/
        /* 3791 WRITE BLOCK                                              */
        /*---------------------------------------------------------------*/
        case 0x05:
            logdump("WRITE BLOCK", dev, iobuf, count);
            *residual=0;
            *unitstat=CSW_CE|CSW_DE;
            break;

        /*---------------------------------------------------------------*/
        /* WRITE type CCWs                                               */
        /*---------------------------------------------------------------*/
        case 0x09:   /* WRITE BREAK */
        case 0x01:   /* WRITE */
            dev->commadpt->write_ccw_count++;
            dev->commadpt->unack_attn_count = 0;
            logdump("WRITE", dev, iobuf, count);
            if (dev->commadpt->emu3791 && (iobuf[4] & 0xf0) == 0x20)
                th_remap(MAP_FID2_FID1, iobuf, dev->commadpt->locsuba);
            if ((iobuf[0] & 0xf0) == 0x10) {  // if FID1
                if (dev->commadpt->debug_sna)
                    format_sna(iobuf, "WR", dev->ssid, dev->devnum);
                make_sna_response(iobuf, dev->commadpt);
                make_sna_requests(iobuf, dev->commadpt);
            }
            *residual = 0;
            *unitstat = CSW_CE | CSW_DE;
            break;

        /*---------------------------------------------------------------*/
        /* CCWs to be treated as NOPs                                    */
        /*---------------------------------------------------------------*/
        case 0x03:   /* NOP */
        case 0x93:   /* RESTART */
        case 0x31:   /* WS0 */
        case 0x51:   /* WS1 */
        case 0x32:   /* RS0 */
        case 0x52:   /* RS1 */
            *residual=count;
            *unitstat=CSW_CE|CSW_DE;
            break;

        default:
        /*---------------------------------------------------------------*/
        /* INVALID OPERATION                                             */
        /*---------------------------------------------------------------*/
            /* Set command reject sense byte, and unit check status */
            *unitstat=CSW_CE+CSW_DE+CSW_UC;
            dev->sense[0]=SENSE_CR;
            break;

    }
    release_lock(&dev->commadpt->lock);
}


/*---------------------------------------------------------------*/
/* DEVICE FUNCTION POINTERS                                      */
/*---------------------------------------------------------------*/

static DEVHND com3705_device_hndinfo =
{
    &commadpt_init_handler,        /* Device Initialization      */
    &commadpt_execute_ccw,         /* Device CCW execute         */
    &commadpt_close_device,        /* Device Close               */
    &commadpt_query_device,        /* Device Query               */
    NULL,                          /* Device Extended Query      */
    NULL,                          /* Device Start channel pgm   */
    NULL,                          /* Device End channel pgm     */
    NULL,                          /* Device Resume channel pgm  */
    NULL,                          /* Device Suspend channel pgm */
    &commadpt_halt_or_clear,       /* Device Halt channel pgm    */
    NULL,                          /* Device Read                */
    NULL,                          /* Device Write               */
    NULL,                          /* Device Query used          */
    NULL,                          /* Device Reserve             */
    NULL,                          /* Device Release             */
    NULL,                          /* Device Attention           */
    commadpt_immed_command,        /* Immediate CCW Codes        */
    NULL,                          /* Signal Adapter Input       */
    NULL,                          /* Signal Adapter Output      */
    NULL,                          /* Signal Adapter Sync        */
    NULL,                          /* Signal Adapter Output Mult */
    NULL,                          /* QDIO subsys desc           */
    NULL,                          /* QDIO set subchan ind       */
    NULL,                          /* Hercules suspend           */
    NULL                           /* Hercules resume            */
};


/* Libtool static name colision resolution */
/* note : lt_dlopen will look for symbol & modulename_LTX_symbol */

#if defined( HDL_USE_LIBTOOL )
#define hdl_ddev hdt3705_LTX_hdl_ddev
#define hdl_depc hdt3705_LTX_hdl_depc
#define hdl_reso hdt3705_LTX_hdl_reso
#define hdl_init hdt3705_LTX_hdl_init
#define hdl_fini hdt3705_LTX_hdl_fini
#endif


HDL_DEPENDENCY_SECTION;
{
    HDL_DEPENDENCY(HERCULES);
    HDL_DEPENDENCY(DEVBLK);
    HDL_DEPENDENCY(SYSBLK);
}
END_DEPENDENCY_SECTION;


#if defined(WIN32) && !defined(HDL_USE_LIBTOOL) && !defined(_MSVC_)
#undef sysblk
HDL_RESOLVER_SECTION;
{
    HDL_RESOLVE_SYMPTR( psysblk, sysblk );
}
END_RESOLVER_SECTION;
#endif /* defined(WIN32)... */


HDL_DEVICE_SECTION;
{
    HDL_DEVICE(3705, com3705_device_hndinfo );
}
END_DEVICE_SECTION;
