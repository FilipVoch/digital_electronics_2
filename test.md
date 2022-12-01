    int CursorX = 0;
    int CursorY = 0;
    
    if (channel == 0) //osa x
    {
    value = ADC;
    if (value > 600)
    {
    CursorX++;
    if (CursorX > 6)
    {
    CursorX = 0;
    }
    lcd_gotoxy(CursorX, CursorY);
    }
    else if (value < 400)
    {
    CursorX--;
    if (CursorX < 0)
    {
    CursorX = 6;
    }
    lcd_gotoxy(CursorX, CursorY);
    }
    else
    {
    lcd_gotoxy(CursorX, CursorY);
    }
    channel = 1;
    }
    
    else if (channel == 1) //osa y
    
    {
    value = ADC;
    if (value > 600)
    {
    CursorY != CursorY;
    lcd_gotoxy(CursorX, CursorY);
    }
    else if (value < 400)
    {
    CursorY != CursorY;
    lcd_gotoxy(CursorX, CursorY);
    }
    else
    {
    lcd_gotoxy(CursorX, CursorY);
    }
    channel = 0;
    }
