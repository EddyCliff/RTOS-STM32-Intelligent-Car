BYTE hidSendDataWaitTilDone (BYTE* dataBuf,
    WORD size,
    BYTE intfNum,
    ULONG ulTimeout);
BYTE hidSendDataInBackground (BYTE* dataBuf,
    WORD size,
    BYTE intfNum,
    ULONG ulTimeout);
WORD hidReceiveDataInBuffer(BYTE *,WORD,BYTE);

BYTE cdcSendDataWaitTilDone (BYTE* dataBuf,
    WORD size,
    BYTE intfNum,
    ULONG ulTimeout);
BYTE cdcSendDataInBackground (BYTE* dataBuf,
    WORD size,
    BYTE intfNum,
    ULONG ulTimeout);
WORD cdcReceiveDataInBuffer(BYTE *,WORD,BYTE);

BYTE phdcSendDataWaitTilDone (BYTE* dataBuf,
    WORD size,
    BYTE intfNum,
    ULONG ulTimeout);
BYTE phdcSendDataInBackground (BYTE* dataBuf,
    WORD size,
    BYTE intfNum,
    ULONG ulTimeout);
WORD phdcReceiveDataInBuffer(BYTE *,WORD,BYTE);
