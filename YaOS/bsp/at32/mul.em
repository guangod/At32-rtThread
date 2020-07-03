macro MultiLineComment()
{
    hwnd = GetCurrentWnd()
    selection = GetWndSel(hwnd)
    LnFirst = GetWndSelLnFirst(hwnd)     //鍙栭琛岃鍙?
    LnLast = GetWndSelLnLast(hwnd)     //鍙栨湯琛岃鍙?
    hbuf = GetCurrentBuf()
  
    if(GetBufLine(hbuf, 0) =="//magic-number:tph85666031"){
        stop
    }
  
    Ln = Lnfirst
    buf = GetBufLine(hbuf, Ln)
    len =strlen(buf)
  
    while(Ln <= Lnlast) {
        buf = GetBufLine(hbuf, Ln) //鍙朙n瀵瑰簲鐨勮
        if(buf ==""){                   //璺宠繃绌鸿
            Ln = Ln + 1
            continue
        }
  
        if(StrMid(buf, 0, 1) =="/") {       //闇€瑕佸彇娑堟敞閲?闃叉鍙湁鍗曞瓧绗︾殑琛?
            if(StrMid(buf, 1, 2) =="/"){
                PutBufLine(hbuf, Ln, StrMid(buf, 2, Strlen(buf)))
            }
        }
  
        if(StrMid(buf,0,1) !="/"){          //闇€瑕佹坊鍔犳敞閲?
            PutBufLine(hbuf, Ln, Cat("//", buf))
        }
        Ln = Ln + 1
    }
  
    SetWndSel(hwnd, selection)
}