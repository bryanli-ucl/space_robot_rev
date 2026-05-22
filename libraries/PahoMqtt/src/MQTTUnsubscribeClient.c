/*******************************************************************************
 * Copyright (c) 2014 IBM Corp.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at
 *   http://www.eclipse.org/org/documents/edl-v10.php
 *******************************************************************************/

#include "MQTTPacket.h"

#include <stddef.h>

int MQTTSerialize_unsubscribeLength(int count, MQTTString topicFilters[]) {
    int i;
    int len = 2; /* packetid */

    for (i = 0; i < count; ++i) {
        len += 2 + MQTTstrlen(topicFilters[i]);
    }

    return len;
}

int MQTTSerialize_unsubscribe(unsigned char* buf,
                              int buflen,
                              unsigned char dup,
                              unsigned short packetid,
                              int count,
                              MQTTString topicFilters[]) {
    unsigned char* ptr = buf;
    MQTTHeader header = {0};
    int rem_len = 0;
    int rc = -1;
    int i;

    if (MQTTPacket_len(rem_len = MQTTSerialize_unsubscribeLength(count, topicFilters)) > buflen) {
        goto exit;
    }

    header.bits.type = UNSUBSCRIBE;
    header.bits.dup = dup;
    header.bits.qos = 1;
    writeChar(&ptr, header.byte);

    ptr += MQTTPacket_encode(ptr, rem_len);
    writeInt(&ptr, packetid);

    for (i = 0; i < count; ++i) {
        writeMQTTString(&ptr, topicFilters[i]);
    }

    rc = (int)(ptr - buf);

exit:
    return rc;
}

int MQTTDeserialize_unsuback(unsigned short* packetid, unsigned char* buf, int buflen) {
    MQTTHeader header = {0};
    unsigned char* curdata = buf;
    unsigned char* enddata = NULL;
    int rc = 0;
    int mylen = 0;

    header.byte = readChar(&curdata);
    if (header.bits.type != UNSUBACK) {
        goto exit;
    }

    curdata += MQTTPacket_decodeBuf(curdata, &mylen);
    enddata = curdata + mylen;
    if (enddata - curdata < 2 || enddata > buf + buflen) {
        goto exit;
    }

    *packetid = readInt(&curdata);
    rc = 1;

exit:
    return rc;
}
